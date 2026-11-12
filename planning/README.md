# Часть 2: сравнение VAMP RRTConnect и OMPL RRTConnect + Coal

## Сборка

Из корня репозитория:

```sh
sudo apt-get update
sudo apt-get install -y build-essential cmake git python3-venv pkg-config

python3 -m venv .venv
.venv/bin/python -m pip install --upgrade pip
.venv/bin/python -m pip install -r requirements-lock.txt
PREFIX=$(.venv/bin/python -c 'import sysconfig; print(sysconfig.get_paths()["purelib"] + "/cmeel.prefix")')

cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH="$PREFIX" \
  -DMPB_BUILD_BENCHMARKS=ON -DMPB_BUILD_PLANNING=ON
cmake --build build --target build_planning -j 4
mkdir -p results/planning
./build/planning_benchmark --benchmark_min_time=1x --benchmark_repetitions=5 \
  --benchmark_out=results/planning/benchmark.json --benchmark_out_format=json
```

## Конфигурация бенчмарков

Параметры определены в [`core/config.hpp`](core/config.hpp):

```cpp
struct PlanningConfig {
  float range = 1.0F;
  static constexpr uint32_t motion_resolution = 32;
  double validation_step = 1.0 / 128.0;
  size_t max_iterations = 100000;
  size_t max_samples = 100000;
};

struct ProblemConfig {
  std::array<float, 3> lower{-0.9F, -0.9F, 0.3F};
  std::array<float, 3> upper{0.9F, 0.9F, 1.8F};
  float min_radius = 0.05F;
  float max_radius = 0.12F;
  double min_endpoint_distance = 1.5;
  uint32_t max_generation_attempts = 100000;
};
```

`PlanningConfig`: `range` — длина максимального нового ребра в L2 норме пространства решений(6-мерные векторы углов);
`motion_resolution` — плотность проверки движения(кол-во шагов на радиану); `validation_step` — шаг проверки соединения. `max_iterations` и `max_samples` ограничивают поиск VAMP.

`ProblemConfig`: `lower` и `upper` - ограничение области препятствий. Начальная и конечная
конфигурации разделены минимум на `min_endpoint_distance` в L2 норме.

Существование пути не гарантируется.

### Визуализация найденных путей

Для экспорта найденего пути надо указать путь до .json файла через `--export`

```sh
mkdir -p results/planning/trajectories
./build/planning_benchmark vamp 100 42 42 \
  --export results/planning/trajectories/vamp_1.json
./build/planning_benchmark ompl 100 42 42 \
  --export results/planning/trajectories/ompl_1.json
python3 planning/scripts/build_planning_viewer.py --results results/planning
```

Потом можно открыть `results/planning/viewer.html` в браузере и посмотреть визуализацию.

### perf и flamegraphs

```sh
perf record -e cpu-clock:u -F 499 --call-graph dwarf,16384 -o results/planning/vamp.data -- ./build/planning_benchmark vamp 100 42 42 > results/planning/vamp_profile.json
perf script -i results/planning/vamp.data -F comm,pid,tid,time,event,ip,sym,dso | ~/FlameGraph/stackcollapse-perf.pl | ~/FlameGraph/flamegraph.pl > results/planning/vamp.svg
perf record -e cpu-clock:u -F 499 --call-graph dwarf,16384 -o results/planning/ompl.data -- ./build/planning_benchmark ompl 100 42 42 > results/planning/ompl_profile.json
perf script -i results/planning/ompl.data -F comm,pid,tid,time,event,ip,sym,dso | ~/FlameGraph/stackcollapse-perf.pl | ~/FlameGraph/flamegraph.pl > results/planning/ompl.svg
```

## Выводы

Как и ожидалось, VAMP показал себя значительно лучше OMPL. Но если честно, я даже не ожидал, что настолько. Если смотреть на флеймграфы, то в VAMP больше всего времени уходит на валидацию пути. Это дополнительная проверка, которая проверяет корректность найденого пути - с меньшим шагом и с одинаковым collide кодом для VAMP и OMPL.

Чисто по времени исполнения ускорение на 100 сферах примерно в 6.5 раз, похоже на числа, указанные в статье. Но стоит проверить отдельно, уже без запуска валидации. При этом на моих запусках, OMPL иногда находил более короткий путь, но по моему мнению более чем 6 кратное ускорение работы компенсирует чуть менее оптимальный путь.

## Результаты

Из [файла с результатами](../results/planning/benchmark.json)

Время указано медианное, в микросекундах

### VAMP

| N препятствий | Время инициализации | Время работы |
| ---: | ---: | ---: |
| 1 | 3.986 | 59221 | 
| 10 | 5.419 | 61345 |
| 100 | 13.111 | 266895 |

### OMPL

| N препятствий | Время инициализации | Время работы |
| ---: | ---: | ---: |
| 1 | 77.21 | 83699 | 
| 10 | 84.38 | 143996 |
| 100 | 118.374 | 1776130 |