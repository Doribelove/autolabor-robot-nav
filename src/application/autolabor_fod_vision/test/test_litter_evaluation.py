import copy
import unittest
from autolabor_fod_vision.litter_evaluation import evaluate


class EvaluationTest(unittest.TestCase):
    def setUp(self):
        self.audit=dict(schema_version=1,valid=True,session_id='test',source_stamp_ns=10000000000,
                        filter_sha256='test-config',implementation_sha256='test-implementation',decisions=[
                            dict(bbox=[10,10,30,30],state='uncertain'),
                            dict(bbox=[100,100,200,250],state='environment')])
        self.labels=dict(exhaustive_trash_labels=True,frames=[
            dict(session_id='test',source_stamp_ns=10000000000,trash_boxes=[[10,10,30,30]])])

    def test_environment_removed_but_unknown_target_counts_as_retained(self):
        report=evaluate([self.audit],self.labels)
        self.assertEqual(report['removed_false_positives'],1)
        self.assertEqual(report['additional_false_negatives'],0)
        self.assertEqual(report['counts']['filtered']['recall'],1.)

    def test_filter_induced_miss_reported(self):
        self.audit['decisions'][0]['state']='environment'
        report=evaluate([self.audit],self.labels)
        self.assertEqual(report['additional_false_negatives'],1)

    def test_detector_miss_is_not_hidden_by_filter_statistics(self):
        self.labels['frames'][0]['trash_boxes'].append([400,100,430,130])
        report=evaluate([self.audit],self.labels)
        self.assertEqual(report['counts']['raw']['fn'],1)
        self.assertEqual(report['counts']['filtered']['fn'],1)

    def test_duplicate_predictions_only_match_once(self):
        self.audit['decisions'].append(copy.deepcopy(self.audit['decisions'][0]))
        report=evaluate([self.audit],self.labels)
        self.assertEqual(report['counts']['filtered']['tp'],1)
        self.assertEqual(report['counts']['filtered']['fp'],1)

    def test_missing_frame_duplicate_frame_or_partial_labels_rejected(self):
        for audits,labels in (([],self.labels),([self.audit,self.audit],self.labels),
                              ([self.audit],dict(self.labels,exhaustive_trash_labels=False))):
            with self.assertRaises(ValueError):evaluate(audits,labels)

    def test_negative_only_frame_has_no_invented_recall(self):
        self.labels['frames'][0]['trash_boxes']=[]
        report=evaluate([self.audit],self.labels)
        self.assertIsNone(report['counts']['filtered']['recall'])

    def test_different_implementations_cannot_be_pooled(self):
        other=copy.deepcopy(self.audit);other['source_stamp_ns']+=100000000
        other['implementation_sha256']='different-code'
        frame=copy.deepcopy(self.labels['frames'][0]);frame['source_stamp_ns']=other['source_stamp_ns']
        self.labels['frames'].append(frame)
        with self.assertRaises(ValueError):evaluate([self.audit,other],self.labels)


if __name__=='__main__':unittest.main()
