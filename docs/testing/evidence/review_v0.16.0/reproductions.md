# Reproduktionen des Reviews 0.16.0

Diese Prüfprogramme wurden außerhalb des Repositorys ausgeführt. Sie sind hier
als Text dokumentiert und gehören weder zum Bibliotheksbuild noch zur produktiven
Testsuite. Befunde und Grenzen stehen im [Review](../../../architecture/architecture_review_v0.16.0.md).

Der Original-Checkout lag unter `/workspace/robot-motion-cpp`, Scratch unter
`/tmp/motion-review-v0.16.0`. Bei anderem Checkout diese Pfade in den folgenden
Originaldateien ersetzen. Vorhandene Release-/Werkzeugbuilds stammten aus 0.16.0.

F13 benötigt die vier dargestellten Dateien unter `odr/`; beide Binaries werden
mit denselben Archiven und nur anderer Linkreihenfolge erzeugt:

```sh
cmake -S /tmp/motion-review-v0.16.0/odr -B /tmp/motion-review-v0.16.0/odr-build -DCMAKE_BUILD_TYPE=Debug -DCMAKE_CXX_FLAGS_DEBUG="-O0 -g1 -fno-inline"
cmake --build /tmp/motion-review-v0.16.0/odr-build --parallel 2
/tmp/motion-review-v0.16.0/odr-build/gantry_first
/tmp/motion-review-v0.16.0/odr-build/serial_first
```

F14 übersetzt die **unveränderte** Implementierungsdatei erneut mit Instrumentierung.
Der Eigen-Includepfad muss auf das lokale entpackte Archiv zeigen. Dieser Lauf soll
fehlschlagen; die anderen Reviewprüfungen bleiben davon getrennt.

```sh
c++ -std=c++20 -O1 -g1 -DNDEBUG -fsanitize=address,undefined -fno-omit-frame-pointer -I include -isystem build-all/_deps/motion_eigen-src /tmp/motion-review-v0.16.0/empty_differential.cpp src/kinematics/inverse_differential.cpp -o /tmp/motion-review-v0.16.0/empty_differential
ASAN_OPTIONS=detect_leaks=0 UBSAN_OPTIONS=halt_on_error=1:print_stacktrace=1 /tmp/motion-review-v0.16.0/empty_differential
```

F15 ist headerbasiert:

```sh
c++ -std=c++20 -O1 -I include /tmp/motion-review-v0.16.0/late_cancel.cpp -o /tmp/motion-review-v0.16.0/late_cancel
/tmp/motion-review-v0.16.0/late_cancel
```

Für F16/F17 zunächst den getrennten Viewer bauen und eine Basisspur erzeugen,
oder den Pfad im Prüfskript auf eine vorhandene vollständige Spur ändern:

```sh
cmake -S tools/motion_viewer -B build-motion-viewer -DCMAKE_BUILD_TYPE=Release
cmake --build build-motion-viewer --parallel 2
mkdir -p /tmp/motion-resources/traces
./build-motion-viewer/motion_viewer_exporter programs/examples/ur5e_demo.yaml /tmp/motion-resources/traces/ur5e_demo-10.json 10
python3 /tmp/motion-review-v0.16.0/probe_tools.py
```

Der GC-Test patcht ausschließlich den Löschaufruf während sechs kurzer Läufe.
Danach wird der Patch beendet und der temporäre Serviceordner aufgeräumt. Es wird
kein Laufwerk gefüllt und keine bestehende Projektdatei gelöscht.

## F13: Parent-CMake

Originaldatei: `odr/CMakeLists.txt`.

```cmake
cmake_minimum_required(VERSION 3.20)
project(mixed_components LANGUAGES CXX)
set(MOTION_BUILD_TESTS OFF)
set(MOTION_BUILD_EXAMPLES OFF)
set(MOTION_COMPONENTS planning serial_kinematics)
add_subdirectory(/workspace/robot-motion-cpp motion)
add_library(gantry_user STATIC gantry.cpp)
target_link_libraries(gantry_user PRIVATE RobotMotion::planning)
add_library(serial_user STATIC serial.cpp)
target_link_libraries(serial_user PRIVATE RobotMotion::serial_kinematics)
foreach(order gantry_first serial_first)
  add_executable(${order} main.cpp)
endforeach()
target_link_libraries(gantry_first PRIVATE gantry_user serial_user)
target_link_libraries(serial_first PRIVATE serial_user gantry_user)
```

## F13: Gantry-Verbraucher

Originaldatei: `odr/gantry.cpp`.

```cpp
#include "motion/kinematics/interval_kinematics.hpp"
bool gantry() {
  motion::CartesianGantry model;
  return motion::enclosure::evaluate(model, std::vector<motion::enclosure::Jet>(3)).position[0].d[0].lo == 0;
}
```

## F13: Serial-Verbraucher

Originaldatei: `odr/serial.cpp`.

```cpp
#include "motion/kinematics/interval_kinematics.hpp"
#include <string>
std::string serial() {
  using namespace motion;
  SerialChainKinematics model({{"x","base","x",SerialJointType::Prismatic,{}, {1,0,0},-2,2,1}});
  try {
    const auto r = enclosure::evaluate(model, {enclosure::Jet(.1)});
    return "ready: " + std::to_string(r.position[0].d[0].lo);
  } catch(const std::exception &e) { return e.what(); }
}
```

## F13: Hauptprogramm

Originaldatei: `odr/main.cpp`.

```cpp
#include <iostream>
#include <string>
bool gantry();
std::string serial();
int main() { std::cout << "gantry=" << gantry() << ", serial=" << serial() << '\n'; }
```

## F14: Nullachsenprovider

Originaldatei: `empty_differential.cpp`.

```cpp
#include "motion/kinematics/inverse_differential.hpp"
#include <iostream>
struct EmptyModel final : motion::IDifferentialKinematics {
  motion::KinematicJointLimits limits;
  std::size_t dof() const override { return 0; }
  motion::Pose fk(const motion::Vector &) const override { return {}; }
  std::optional<motion::Vector> ik(const motion::Pose &,const motion::Vector &) const override { return std::nullopt; }
  std::vector<motion::Vector> jacobian(const motion::Vector &) const override { return std::vector<motion::Vector>(6); }
  const motion::KinematicJointLimits &position_limits() const noexcept override { return limits; }
  motion::SingularityInfo singularity(const motion::Vector &) const override { return {}; }
  motion::KinematicDerivatives differentiate(const motion::Vector &,const motion::Vector &,const motion::Vector &,const motion::Vector &) const override { return {}; }
};
int main(){ auto result=motion::inverse_differential(EmptyModel{}, {}, {}, {}); std::cout << bool(result) << ':' << result.diagnostic << '\n'; }
```

## F15: Stop im letzten Constraint

Originaldatei: `late_cancel.cpp`.

```cpp
#include "motion/planning/envelope_constraints.hpp"
#include <iostream>
struct StopConstraint final:motion::IEnvelopeConstraint{
  std::stop_source *source;
  explicit StopConstraint(std::stop_source &s):source(&s){}
  bool admit(const motion::IKinematics &,const std::vector<motion::enclosure::Jet> &,const motion::enclosure::Result &)const override { source->request_stop(); return true; }
};
int main(){using namespace motion;std::stop_source stop;PreparationContext context({},stop.get_token());
 JointLimits limits{Vector(3,-1),Vector(3,1),Vector(3,1),Vector(3,1),Vector(3,1)};
 auto p=plan_boundary_quintic(limits,{{.1,.2,.3},Vector(3),Vector(3),0},{{.1,.2,.3},Vector(3),Vector(3)},1);
 auto result=check_polynomial_constraints(CartesianGantry{},*p.trajectory,{1,1,1,1,1,1},{std::make_shared<StopConstraint>(stop)},{16,100000,&context});
 std::cout<<"stop_requested="<<stop.stop_requested()<<", admitted="<<bool(result)<<", context_failure="<<int(context.failure())<<'\n';}
```

## F16/F17: Spur und GC-Fehler

Originaldatei: `probe_tools.py`.

```python
from pathlib import Path
import csv,io,json,sys
from unittest.mock import patch
sys.path.insert(0, str(Path('/workspace/robot-motion-cpp/tools/motion_viewer')))
sys.path.insert(0, str(Path('/workspace/robot-motion-cpp/tools/motion_viewer/tests')))
from create_viewer import prepare_data, load_export
from editor_server import SimulationService
from test_program_editor import request_for, EXPORTER

exported=load_export('/tmp/motion-resources/traces/ur5e_demo-10.json')
rows=list(csv.reader(io.StringIO(exported['trace_csv'])))
index={s:i for i,s in enumerate(rows[0])}
for row in rows[1:]:
    row[index['segment']]='1'
    row[index['source_step']]='999'
text=io.StringIO(newline='');writer=csv.writer(text,lineterminator='\n');writer.writerows(rows)
exported['trace_csv']=text.getvalue()
result=prepare_data(exported,max_frames=40,visuals='none')
report={'trace':{'accepted':True,'defined_segments':len(result['program']['segments']),
                'reported_segments':sorted({row[index['segment']] for row in result['frames']}),
                'reported_source_steps':sorted({row[index['source_step']] for row in result['frames']}),
                'limit_violations':result['sample_limit_violations']}}
service=SimulationService(EXPORTER)
try:
    with patch('editor_server.shutil.rmtree',side_effect=OSError('persistent cleanup failure')):
        for _ in range(6): service.simulate(request_for())
    report['cleanup']={'published_results':len(service._results),'pending_cleanup':len(service._expired_results),
                       'result_directories':len(list(service.folder.iterdir()))}
finally: service.close()
print(json.dumps(report,indent=2))
```
