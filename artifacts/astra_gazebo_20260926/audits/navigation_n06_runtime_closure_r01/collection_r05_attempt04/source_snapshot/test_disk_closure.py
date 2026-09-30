import importlib.util,json,os,sys,tempfile,unittest
from pathlib import Path
from unittest.mock import patch
SOURCE=Path(__file__).resolve().parents[1]/'build_disk_closure.py'
spec=importlib.util.spec_from_file_location('n06_disk',SOURCE);d=importlib.util.module_from_spec(spec);spec.loader.exec_module(d)
class DiskParsingTests(unittest.TestCase):
 def test_ldd_transitive_loader_and_virtual(self):
  rows,virtual=d.parse_ldd('linux-vdso.so.1 (0x1234)\nlibx.so.1 => /lib/x.so.1 (0xabcd)\n/lib64/ld-linux-x86-64.so.2 (0x0011)\n')
  self.assertEqual(len(rows),2);self.assertEqual(rows[0]['soname'],'libx.so.1');self.assertEqual(len(virtual),1)
 def test_unresolved_and_unrecognized_lines_fail(self):
  for s in ('libx => not found','libx /tmp/x (unknown)'):
   with self.assertRaises(ValueError):d.parse_ldd(s)
 def test_static_and_needed_parser(self):
  self.assertEqual(d.parse_ldd('statically linked'),([],[]));self.assertEqual(d.dynamic_needed('0x1 (NEEDED) Shared library: [libc.so.6]'),['libc.so.6'])
 def test_inherited_rpath_requires_parent_table_containing_current_library(self):
  parent={'file':{'path':'/root.so','sha256':'1'*64},'resolved':[{'soname':'a.so','alias':'/a.so','binding':{'path':'/a.so'}},{'soname':'b.so','alias':'/private/b.so','binding':{'path':'/private/b.so'}}]}
  rows,_,missing,contexts=d.inherited_resolution('b.so => not found','/a.so',[parent])
  self.assertEqual(rows,[{'soname':'b.so','alias':'/private/b.so'}]);self.assertEqual(missing,['b.so']);self.assertEqual(contexts,[parent['file']])
  with self.assertRaises(ValueError):d.inherited_resolution('b.so => not found','/unrelated.so',[parent])
  with self.assertRaises(ValueError):d.inherited_resolution('absent.so => not found','/a.so',[parent])
 def test_real_gzserver_alias_uses_canonical_binding(self):
  with tempfile.TemporaryDirectory() as t:
   c=d.Collector(Path(t)/'out');b=c.add('/usr/bin/gzserver','test')
   self.assertEqual(b['path'],str(Path('/usr/bin/gzserver').resolve()));self.assertIn('/usr/bin/gzserver',c.aliases)
 def test_failure_raw_command_retained_and_no_retry_directory(self):
  with tempfile.TemporaryDirectory() as t:
   root=Path(t)/'out';c=d.Collector(root)
   with self.assertRaises(RuntimeError):c.command(['/bin/false'],'negative')
   record=json.loads(next((root/'commands').glob('*.json')).read_text());self.assertNotEqual(record['returncode'],0)
   self.assertTrue(Path(record['stdout']['path']).exists())
   with self.assertRaises(FileExistsError):d.Collector(root)
 def test_safe_environment_excludes_arbitrary_credentials(self):
  with patch.dict(os.environ,{'UNRELATED_PRIVATE_TOKEN':'never_print','AWS_SECRET_ACCESS_KEY':'never_print'}):
   env=d.safe_environment();self.assertNotIn('UNRELATED_PRIVATE_TOKEN',env);self.assertNotIn('AWS_SECRET_ACCESS_KEY',env)
if __name__=='__main__':unittest.main()
