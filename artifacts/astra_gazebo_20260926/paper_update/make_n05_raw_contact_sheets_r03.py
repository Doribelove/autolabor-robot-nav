"""Index native N05 camera frames for actual visual QC; do not alter frames."""
import hashlib
import json
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

S=Path(__file__).resolve().parents[1]
LEDGER=S/'queue/navigation_n05_ledger.json'
OUT=S/'paper_update/raw_qc/navigation_n05_contact_sheets_r03'


def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    ledger=json.loads(LEDGER.read_bytes())
    if ledger['status']!='terminal' or OUT.exists():
        raise ValueError('terminal N05 and new QC identity required')
    rows=[]
    for unit in ledger['units']:
        if unit['status']!='consumed':continue
        root=S/'runs'/unit['run_id']
        receipt=json.loads(Path(unit['receipt']['path']).read_bytes())
        raw_path=root/'collector_raw.json'
        if not raw_path.is_file():
            rows.append(dict(run_id=unit['run_id'],image_status='missing_raw',images=[]));continue
        raw=json.loads(raw_path.read_bytes());frames=[]
        for record in raw.get('images',[]):
            path=Path(record['path'])
            valid=path.is_file() and sha(path)==record['sha256']
            if valid:
                with Image.open(path) as im:
                    valid=im.size==(2560,1440)
            frames.append(dict(path=str(path),sha256=record['sha256'],source_ns=record['source_ns'],valid_native=valid))
        rows.append(dict(run_id=unit['run_id'],seed=unit['seed'],method=unit['method'],
                         task_outcome=receipt['task_outcome'],integrity_pass=receipt['integrity_pass'],
                         image_status='two_valid_native' if len(frames)==2 and all(x['valid_native'] for x in frames) else 'missing_or_invalid_native',
                         images=frames))
    OUT.mkdir(parents=True)
    font=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',22)
    card_w,card_h=480,310
    cards=[]
    for row in rows:
        for idx,frame in enumerate(row['images']):
            card=Image.new('RGB',(card_w,card_h),'white');d=ImageDraw.Draw(card)
            if frame['valid_native']:
                with Image.open(frame['path']) as original:
                    thumb=original.convert('RGB').resize((480,270),Image.LANCZOS)
                card.paste(thumb,(0,0))
            suffix=row['run_id'].split('_b')[-1].split('_')[0]
            method='P' if 'preview' in row['method'] else 'C'
            label=f"b{suffix} {method} {idx+1}/2 {frame['source_ns']/1e9:.2f}s {row['task_outcome']}"
            d.text((6,278),label,fill='black',font=font)
            cards.append(card)
    pages=[]
    for start in range(0,len(cards),24):
        page=Image.new('RGB',(6*card_w,4*card_h),'#e8edf2')
        for offset,card in enumerate(cards[start:start+24]):
            page.paste(card,((offset%6)*card_w,(offset//6)*card_h))
        path=OUT/f'contact_sheet_{start//24+1:02d}.png';page.save(path)
        pages.append(dict(path=str(path),sha256=sha(path),first_card=start,total_cards=min(24,len(cards)-start)))
    manifest=dict(schema='astra_n05_raw_camera_contact_index_v1',release_sha256=ledger['release_sha256'],
                  ledger_sha256=sha(LEDGER),status='generated_pending_actual_view',
                  image_count=len(cards),rows=rows,pages=pages,
                  statement='Contact sheets are downsampled indexes only; all raw native 2560x1440 PNGs retained and SHA-bound. Visual QC must inspect originals for any ambiguity. The r01 indexing attempt failed before any sheet due to unavailable PIL.Image.Resampling; r02 labels clipped at card edges. Both are preserved.')
    (OUT/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    print(json.dumps(dict(status=manifest['status'],runs=len(rows),images=len(cards),pages=len(pages))))


if __name__=='__main__':main()
