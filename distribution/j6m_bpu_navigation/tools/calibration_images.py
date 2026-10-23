#!/usr/bin/env python3
"""Create source-stamped PTQ tensors from an explicitly selected training list."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import sys

os.environ['CUDA_VISIBLE_DEVICES'] = ''
os.environ['YOLO_AUTOINSTALL'] = 'false'
os.environ['OPENBLAS_NUM_THREADS'] = '1'
os.environ['OMP_NUM_THREADS'] = '1'

from production_ptq import contract, digest, write_json


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', choices=list(contract()['models']), required=True)
    parser.add_argument('--images', type=Path, required=True,
                        help='JSON: split=train, samples=[{path,class_name}]')
    parser.add_argument('--ultralytics-root', type=Path, required=True)
    parser.add_argument('--checkpoint', type=Path,
                        help='Pinned classifier best.pt; used to verify its saved transforms')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    data = json.loads(args.images.read_text())
    spec = contract()['models'][args.model]
    samples = data.get('samples', [])
    if data.get('split') != 'train' or len(samples) < spec['minimum_calibration_samples']:
        raise ValueError('Explicitly select at least 20 representative training samples')
    paths, classes = [], set()
    for item in samples:
        path = Path(item['path'])
        if not path.is_absolute():
            path = args.images.resolve().parent / path
        path = path.resolve(strict=True)
        if ({'val', 'valid', 'validation', 'test'} & {p.lower() for p in path.parts}
                or item['class_name'] not in spec['classes']):
            raise ValueError('Held-out paths or unexpected classes in calibration input')
        classes.add(item['class_name'])
        paths.append((path, item['class_name']))
    if classes != set(spec['classes']):
        raise ValueError('Select every material class')
    if args.output.exists() or args.output.is_symlink():
        raise FileExistsError('Use a new calibration output directory')
    root = args.ultralytics_root.resolve(strict=True)
    sys.path.insert(0, str(root))
    import cv2
    import numpy as np
    from PIL import Image
    import ultralytics
    from ultralytics.data.augment import LetterBox, classify_transforms
    import torch
    if root not in Path(ultralytics.__file__).resolve().parents:
        raise ValueError('Loaded a different Ultralytics implementation')
    torch.set_num_threads(1)
    cv2.setNumThreads(1)
    transform = classify_transforms(size=224)
    transform_description = None
    if args.model == 'classifier':
        if not args.checkpoint or digest(args.checkpoint) != spec['checkpoint_sha256']:
            raise ValueError('Supply the pinned production classifier checkpoint')
        model = ultralytics.YOLO(str(args.checkpoint))
        if [str(model.names[i]) for i in range(len(model.names))] != spec['classes']:
            raise ValueError('Classifier checkpoint class order mismatch')
        saved = getattr(model.model, 'transforms', None)
        if saved is None or repr(saved) != repr(transform):
            raise ValueError('Saved classifier preprocessing differs from the draft contract')
        transform = saved
        transform_description = repr(saved)
    letterbox = LetterBox(new_shape=(576, 1024), auto=False, stride=32)
    args.output.mkdir(parents=True)
    records, seen = [], set()
    for index, (path, label) in enumerate(paths):
        frame = cv2.imread(str(path))
        if frame is None:
            raise ValueError('Unreadable training image: ' + str(path))
        if args.model == 'detector_zed':
            frame = letterbox(image=frame)
            tensor = np.ascontiguousarray(frame[:, :, ::-1].transpose(2, 0, 1)[None],
                                           dtype=np.float32) / np.float32(255)
        else:
            rgb = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
            tensor = transform(rgb).unsqueeze(0).numpy().astype(np.float32)
        if list(tensor.shape) != spec['input_shape'] or not np.isfinite(tensor).all():
            raise ValueError('Calibration preprocessing interface mismatch')
        identity = hashlib.sha256(tensor.tobytes()).hexdigest()
        if identity in seen:
            raise ValueError('Duplicate processed calibration image')
        seen.add(identity)
        name = '%05d.npy' % index
        np.save(args.output / name, tensor, allow_pickle=False)
        records.append({'path': name, 'sha256': digest(args.output / name),
                        'class_name': label, 'source': str(path), 'source_sha256': digest(path)})
    write_json(args.output / 'manifest.json', {
        'model': args.model, 'split': 'train', 'preprocess_id': spec['preprocess_id'],
        'ultralytics_version': ultralytics.__version__,
        'augment_source_sha256': digest(root / 'ultralytics/data/augment.py'),
        'classifier_transform': transform_description,
        'samples': records, 'cuda_initialized': torch.cuda.is_initialized()})
    print(json.dumps({'output': str(args.output), 'samples': len(records),
                      'accuracy_accepted': False}))


if __name__ == '__main__':
    main()
