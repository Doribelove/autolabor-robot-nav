"""Static P02 proof: only named telemetry additions differ from frozen P01."""
from pathlib import Path
import hashlib
import xml.etree.ElementTree as ET

PACKAGE = Path(__file__).resolve().parents[1]
OLD = PACKAGE / 'src/astra_physical_ackermann_plugin.cpp'
NEW = PACKAGE / 'src/astra_physical_ackermann_p02_plugin.cpp'


def test_controller_source_exactly_preserved_after_telemetry_removal():
    before = OLD.read_text()
    assert hashlib.sha256(OLD.read_bytes()).hexdigest() == '8b5d45c0943956d13b1229279924a16dfc2565470d7304266800968270d93e98'
    after = NEW.read_text().replace('AstraPhysicalAckermannP02Plugin', 'AstraPhysicalAckermannPlugin')
    for line in ['#include <diagnostic_msgs/DiagnosticArray.h>\n',
                 '#include <diagnostic_msgs/DiagnosticStatus.h>\n',
                 '#include <diagnostic_msgs/KeyValue.h>\n',
                 '    status_pub_=driver.advertise<diagnostic_msgs::DiagnosticArray>("physical_status",1000);\n',
                 '  bool status_clock_fault_logged_{false};\n']:
        assert after.count(line) == 1
        after = after.replace(line, '')
    begin = after.index('  static bool validStatusSourceTime(double now) {')
    end = after.index('  void publish() {', begin)
    after = after[:begin] + after[end:]
    begin = after.index('    if (!validStatusSourceTime(now)) {')
    end = after.index('    if (now-last_publish_', begin)
    after = after[:begin] + after[end:]
    after = after.replace('    const double now=world_->SimTime().Double();\n    if (now-last_publish_',
                          '    const double now=world_->SimTime().Double();if (now-last_publish_')
    assert after.count('      publishPhysicalStatus(now);\n') == 1
    after = after.replace('      publishPhysicalStatus(now);\n', '')
    assert after.count('    publishPhysicalStatus(now);\n') == 1
    after = after.replace('    publishPhysicalStatus(now);\n', '')
    after = after.replace('metadata_pub_,status_pub_;', 'metadata_pub_;')
    assert after == before


def test_status_schema_queue_source_time_and_failure_paths():
    text = NEW.read_text()
    method = text[text.index('  void publishPhysicalStatus'):text.index('  void publish()')]
    assert 'if (!validStatusSourceTime(now)) return;' in method
    assert 'message.header.stamp=stamp(now)' in method
    assert 'status.name="astra_physical_ackermann";status.hardware_id="simulation-only"' in method
    assert 'status.level=fault_?diagnostic_msgs::DiagnosticStatus::ERROR:' in method
    assert 'timed_out_?diagnostic_msgs::DiagnosticStatus::WARN:diagnostic_msgs::DiagnosticStatus::OK' in method
    for key, value in [('backend_fault', 'fault_'), ('command_timeout', 'timed_out_'),
                       ('brake', 'brake_'), ('emergency', 'emergency_')]:
        assert f'add("{key}",{value});' in method
    assert 'item.value=value?"true":"false"' in method
    assert method.count('message.status.push_back(status)') == 1
    assert 'DiagnosticArray>("physical_status",1000)' in text
    publish = text[text.index('  void publish()'):text.index('  void publishMetadata()')]
    clock = publish[publish.index('    if (!validStatusSourceTime(now))'):publish.index('    if (now-last_publish_')]
    assert 'fault_=true;pending_.clear()' in clock and 'fault_pub_.publish(flag)' in clock
    assert 'source_status_omitted=true' in clock and 'return;' in clock
    assert 'stamp(' not in clock and 'publishPhysicalStatus' not in clock
    finite = publish[publish.index('    if (!std::isfinite(now)||!finiteBaseState()'):publish.index('    const auto pose=')]
    assert 'fault_=true;pending_.clear()' in finite and 'publishPhysicalStatus(now)' in finite and 'return;' in finite
    assert publish.count('publishPhysicalStatus(now);') == 2
    assert 'ConnectWorldUpdateEnd' in text and 'publishRate",50.' in text


def test_xacro_launch_only_library_entrypoints_changed():
    pairs = [
        ('urdf/m2_physical.urdf.xacro', 'urdf/m2_physical_p02.urdf.xacro',
         [('m2_physical_p02_gazebo', 'm2_physical_gazebo')]),
        ('urdf/m2_physical_gazebo.xacro', 'urdf/m2_physical_p02_gazebo.xacro',
         [('m2_physical_p02_gazebo', 'm2_physical_gazebo'),
          ('name="astra_physical_ackermann_p02" filename="libastra_physical_ackermann_p02_plugin.so"',
           'name="astra_physical_ackermann" filename="libastra_physical_ackermann_plugin.so"')]),
        ('launch/spawn_m2_physical.launch', 'launch/spawn_m2_physical_p02.launch',
         [('m2_physical_p02.urdf.xacro', 'm2_physical.urdf.xacro')])]
    for old, new, substitutions in pairs:
        before = (PACKAGE/old).read_text();after = (PACKAGE/new).read_text()
        ET.fromstring(after)
        for x, y in substitutions:
            assert x in after
            after = after.replace(x,y)
        assert before == after
