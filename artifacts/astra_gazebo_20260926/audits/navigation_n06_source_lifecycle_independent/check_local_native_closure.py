"""Exact active custom-library local include and xacro closure, no compilation."""
import hashlib,json,re,xml.etree.ElementTree as ET
from pathlib import Path
A=Path(__file__).resolve().parent;S=A.parents[1];W=S/'workspace';G=W/'src/simulation/m2_gazebo'
def bind(p):
 p=Path(p).resolve();return {'path':str(p),'sha256':hashlib.sha256(p.read_bytes()).hexdigest()}
roster_path=S/'audits/navigation_n06_candidate_roster_r05_r03/resources.json';assert bind(roster_path)['sha256']=='97db74b3d9fcf283b249b66fd1c432a6e5c80ee8b5ec9559669a00dd87ace15a'
roster=json.loads(roster_path.read_text());index={r['path']:r for r in roster['resources']}
names=('astra_physical_ackermann_p05_plugin','astra_dense_evidence_plugin','astra_physics_metadata_p03_plugin')
roots=[G/'src'/(name+'.cpp') for name in names];todo=list(roots);seen=set();edges=[];external=[]
while todo:
 path=todo.pop().resolve()
 if path in seen:continue
 seen.add(path)
 for lineno,line in enumerate(path.read_text().splitlines(),1):
  if not re.match(r'\s*#\s*include\b',line):continue
  found=re.fullmatch(r'\s*#\s*include\s*([<"])([^>"\n]+)[>"]\s*',line)
  assert found,(str(path),lineno,line)
  delim,inc=found.groups();candidates=[]
  # Known target include roots. Quoted local includes search current source first.
  dirs=([path.parent] if delim=='"' else [])+[G/'include',W/'devel/include']
  for directory in dirs:
   p=directory/inc
   if p.is_file() and p.resolve() not in candidates:candidates.append(p.resolve())
  assert len(candidates)<=1,('ambiguous local include',path,inc,candidates)
  if candidates:
   target=candidates[0];edges.append({'from':str(path),'line':lineno,'include':inc,'to':str(target),'target':bind(target)});todo.append(target)
  else:external.append({'from':str(path),'line':lineno,'include':inc,'scope':'installed C++/Gazebo/ROS SDK or standard headers; not a new reproducible-build claim'})
assert len(seen)==10 and len(edges)==7
missing=[bind(p) for p in sorted(seen) if str(p) not in index];assert len(missing)==5
for p in seen:
 if str(p) in index:assert bind(p)==index[str(p)]

cmake=G/'CMakeLists.txt';text=cmake.read_text();targets=[]
assert not re.search(r'(?im)^\s*(include|add_subdirectory)\s*\(',text)
for name in names:
 source='src/'+name+'.cpp'
 assert re.search(r'add_library\(\s*'+re.escape(name)+r'\s+'+re.escape(source)+r'\s*\)',text)
 assert 'add_dependencies('+name+' ${catkin_EXPORTED_TARGETS})' in text
 assert 'target_link_libraries('+name+' ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})' in text
 targets.append({'target':name,'source':bind(G/source),'dependency_scope':'catkin exported generated messages; external linked-library bytes remain separately qualified'})
assert bind(cmake)==index[str(cmake)]
xacro_root=G/'urdf/m2_physical_p05.urdf.xacro';todo=[xacro_root];xseen=set();xedges=[]
while todo:
 p=todo.pop().resolve()
 if p in xseen:continue
 xseen.add(p);tree=ET.fromstring(p.read_text())
 for node in tree.iter():
  if node.tag.rsplit('}',1)[-1]!='include':continue
  value=node.attrib['filename'];prefix='$(find m2_gazebo)/';assert value.startswith(prefix)
  target=(G/value[len(prefix):]).resolve();assert target.is_file()
  xedges.append({'from':str(p),'filename':value,'to':str(target),'target':bind(target)});todo.append(target)
assert len(xseen)==4 and len(xedges)==3 and all(bind(p)==index[str(p)] for p in xseen)
out={'schema':'astra_n06_active_local_native_and_xacro_source_closure_v1','candidate_roster':bind(roster_path),'status':'r03_missing_five_actual_local_headers','active_cpp_roots':[bind(p) for p in roots],'local_include_search_roots':[str(G/'include'),str(W/'devel/include')],
 'local_files':[bind(p) for p in sorted(seen)],'include_edges':edges,'external_installed_sdk_includes':external,'missing':missing,
 'cmake':{'binding':bind(cmake),'targets':targets,'extra_local_cmake_include_or_subdirectory':False,'scope':'Three active native targets only. Other CMake targets/test/install/Python-packaging declarations are not runtime activation or rebuild claims.'},
 'xacro':{'root':bind(xacro_root),'files':[bind(p) for p in sorted(xseen)],'edges':xedges,'all_already_in_r03':True},
 'limits':['Generated workspace headers are read-only existing build outputs bound as source-interface evidence; they are not regenerated or committed.','No installed SDK recursive source-build closure or bit-identical rebuild is claimed; installed ELF/runtime closure is separate.','No CMake, xacro expansion, compiler, ROS or Gazebo was executed.','r02 original eight-item negative remains unchanged; this newly examined transitive local include closure adds five headers to r03.'],
 'source_review_written':False,'execution_authorized':False,'seeds_registered':False,'old_files_modified':False}
path=A/'candidate_r03_recursive_native_missing.json';path.write_text(json.dumps(out,indent=2)+'\n')
print(json.dumps({'report':bind(path),'local_files':len(seen),'edges':len(edges),'missing':missing},indent=2))
