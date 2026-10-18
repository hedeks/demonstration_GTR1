# GTR1 (Гативус Информационно-Сенсомоторный Уровень)

> **Specification-First Driven Development (SDD)**: Данный репозиторий разрабатывается по методологии приоритета спецификации над кодом. Архитектурные границы, инварианты памяти и протоколы взаимодействия сначала строго формализуются в блоке спецификаций (`01_architecture/` – `06_algorithms_ml/`), после чего имплементируются и верифицируются в коде (`src/` и `tests/`).

**GTR1 (Gativus Transformation 1)** — первая макро-трансформация информационно-сенсомоторного уровня когнитивной архитектуры Гативус. Система реализует целенаправленное поведение автономного агента в воспринимаемой физической среде: переход от текущего состояния «где я нахожусь» к целевому состоянию «где находится предмет потребности», непрерывно связывая низкоуровневые сенсорные потоки со смысловыми инвариантами и планами действий.

---

## 🧭 Навигатор по Проекту: Спецификация vs. Код Реализации

В соответствии с подходом SDD, каждому программному компоненту в директории `src/` предшествует детальная математическая и архитектурная спецификация. Ниже приведена сквозная матрица трассируемости:

| Онтологический Домен / Подсистема | 📐 Где Спецификация (Документация и Модели) | 💻 Где Код Реализации (Активная разработка) | 📋 Связанные Issues (Бэклог и Эволюция) |
| :--- | :--- | :--- | :--- |
| **Системная Архитектура & Межпроцессный Обмен (IPC)** | • [01_architecture/TARGET_ARCHITECTURE.md](01_architecture/TARGET_ARCHITECTURE.md)<br>• [01_architecture/ADR_001_MVP_baseline.md](01_architecture/ADR_001_MVP_baseline.md)<br>• [01_architecture/deployment.puml](01_architecture/deployment.puml)<br>• [01_architecture/timing_dfd_package.puml](01_architecture/timing_dfd_package.puml) | • [src/main.cpp](src/main.cpp) (супервизор цикла)<br>• [04_interfaces/ipc_messages.proto](04_interfaces/ipc_messages.proto)<br>• [src/ml_service/ipc_messages_pb2.py](src/ml_service/ipc_messages_pb2.py) | [ISSUE-08](issues.md#issue-08) (IPC/WS безопасность)<br>[ISSUE-09](issues/ISSUE-09_tickrate_sync.md) (Синхронизация) ✅ |
| **DOM3: Пространство (`MP10Space`) & Сенсорный Шлюз (`SLGW`)** | • [01_architecture/TARGET_ARCHITECTURE.md#1](01_architecture/TARGET_ARCHITECTURE.md#11-онтологические-домены-и-место-gtr1-в-архитектуре-гативус)<br>• [05_requirements/ML_and_SLAM_Specs.md](05_requirements/ML_and_SLAM_Specs.md)<br>• [02_data_models/core_classes.puml](02_data_models/core_classes.puml) | • [src/operators/slgw_node.cpp](src/operators/slgw_node.cpp)<br>• [src/core/mp10_space.cpp](src/core/mp10_space.cpp)<br>• [src/core/octomap_manager.cpp](src/core/octomap_manager.cpp)<br>• [src/core/kalman_filter_se3.cpp](src/core/kalman_filter_se3.cpp)<br>• [src/core/depth_filter.cpp](src/core/depth_filter.cpp) | [ISSUE-03](issues.md#issue-03) (Keyframing и Pose Graph)<br>[ISSUE-04](issues.md#issue-04) (Pan-Tilt подвес и саккады OP11) |
| **DOM4: Объекты (`MP11Object`) & Зрение (`CNN1 + CLIP`)** | • [02_data_models/gtr1_structures.proto](02_data_models/gtr1_structures.proto)<br>• [05_requirements/ML_and_SLAM_Specs.md](05_requirements/ML_and_SLAM_Specs.md)<br>• [01_architecture/semantics_gtr.md#44](01_architecture/semantics_gtr.md#44-инварианты-цикла-gtr-и-условия-изоморфизма) | • [src/ml_service/cnn1_worker.py](src/ml_service/cnn1_worker.py)<br>• [src/core/mp11_object.cpp](src/core/mp11_object.cpp)<br>• [src/core/mp11_manager.cpp](src/core/mp11_manager.cpp) | [ISSUE-01](issues.md#issue-01) (CLIP Open-Vocabulary)<br>[ISSUE-02](issues.md#issue-02) (Фоновый тренер TRN1) |
| **DOM5: Мотивация (`MTV1`) & Макро-Планирование (`PRD1`)** | • [01_architecture/TARGET_ARCHITECTURE.md#5](01_architecture/TARGET_ARCHITECTURE.md#5-макро-уровень-предиктор-prd1-как-генератор-токенов-действий-issue-10)<br>• [issues/ISSUE-10_action_token_predictor.md](issues/ISSUE-10_action_token_predictor.md)<br>• [06_algorithms_ml/A_star_cost_function.md](06_algorithms_ml/A_star_cost_function.md) | • [src/operators/prd1_predictor.cpp](src/operators/prd1_predictor.cpp)<br>• [src/core/mtv1_motivator.cpp](src/core/mtv1_motivator.cpp)<br>• [src/core/trl1_log.cpp](src/core/trl1_log.cpp) | [ISSUE-10](issues/ISSUE-10_action_token_predictor.md) (Action-Token Transformer & Stand-off Anchor) 📋 |
| **DOM5: Сплайсинг (`MP12Oprn`) & Микро-Кинематика (`SPL1`)** | • [01_architecture/TARGET_ARCHITECTURE.md#6](01_architecture/TARGET_ARCHITECTURE.md#6-микро-уровень-динамический-сплайсер-spl1-и-моторный-мозжечок-issue-11)<br>• [issues/ISSUE-11_dynamic_spl_kinematic_controller.md](issues/ISSUE-11_dynamic_spl_kinematic_controller.md)<br>• [01_architecture/state_machine.puml](01_architecture/state_machine.puml) | • [src/operators/spl1_splicer.cpp](src/operators/spl1_splicer.cpp)<br>• [src/core/mp12_oprn.cpp](src/core/mp12_oprn.cpp)<br>• [src/core/mp13_blom.cpp](src/core/mp13_blom.cpp) | [ISSUE-11](issues/ISSUE-11_dynamic_spl_kinematic_controller.md) (Dynamic Splicer & Elastic Warping) 📋 |
| **Защитный Барьер & Безопасность (Safety Shield)** | • [01_architecture/TARGET_ARCHITECTURE.md#7](01_architecture/TARGET_ARCHITECTURE.md#7-детерминированный-символический-щит-безопасности-safety-shield)<br>• [05_requirements/Safety_and_Robotics.md](05_requirements/Safety_and_Robotics.md) | • [src/operators/spl1_splicer.cpp](src/operators/spl1_splicer.cpp)<br>• [src/main.cpp](src/main.cpp) (контур E-Stop) | [ISSUE-05](issues.md#issue-05) (3D Clearance Cylinder в UI)<br>[ISSUE-06](issues.md#issue-06) (Функция ошибки TCH1) |
| **Среда Физической Симуляции (PyBullet)** | • [01_architecture/timing_dfd_package.puml](01_architecture/timing_dfd_package.puml)<br>• [issues/ISSUE-09_tickrate_sync.md](issues/ISSUE-09_tickrate_sync.md) | • [src/simulation/pybullet_env.py](src/simulation/pybullet_env.py)<br>• [07_infrastructure/sim_stream_receiver.cpp](07_infrastructure/sim_stream_receiver.cpp) | [ISSUE-09](issues/ISSUE-09_tickrate_sync.md) (Декомпозиция реального времени 60 Гц) ✅ |
| **Телеметрия & 3D Web Dashboard (Three.js)** | • [08_project_management/dashboard_mp10.md](08_project_management/dashboard_mp10.md)<br>• [01_architecture/TARGET_ARCHITECTURE.md#9](01_architecture/TARGET_ARCHITECTURE.md#9-протоколы-взаимодействия-dto-и-синхронизация-issue-09) | • [07_infrastructure/dashboard/app.js](07_infrastructure/dashboard/app.js)<br>• [src/ml_service/ws_bridge.py](src/ml_service/ws_bridge.py)<br>• [07_infrastructure/dashboard/index.html](07_infrastructure/dashboard/index.html) | [ISSUE-05](issues.md#issue-05) (HUD визуализатор)<br>[ISSUE-09](issues/ISSUE-09_tickrate_sync.md) (Атомарный TICK_BUNDLE) ✅ |

---

## 🧠 Онтологическая Парадигма Гативус

Когнитивное ядро функционирует на основе двух встречных, непрерывно замкнутых потоков информации:

```
  =============================================================================
                       RTR (Обратный ход: Развёртка Поведения)
         Цель (MTV1) ──► Токены OPRN (PRD1) ──► Дуги (SPL1) ──► Моторы (OP13)
  =============================================================================
                                      ▲   │
                        Опыт (TRL1)   │   │  Щит Безопасности (Safety Shield)
                                      │   ▼
  =============================================================================
  Физический Мир / PyBullet ──► Шлюз SLGW ──► Воксели MP10 ──► Объекты MP11
                       DTR (Прямой ход: Свертка Восприятия)
  =============================================================================
```

### 1. DTR (Прямой ход — Свёртка и Узнавание)
* **`SLGW` (Sensor Linker Gateway):** Прием сенсорного потока (RGB-D), билатеральная фильтрация шума глубины и фильтрация $SE(3)$-позы (фильтр Калмана).
* **`MP10` (Пространство / DOM3):** Вокселизация сцены в динамическое 3D-дерево OctoMap с разрешением $0.05\text{ м}$.
* **`CNN1 + CLIP` $\to$ `MP11` (Объекты / DOM4):** Детекция объектов на GPU (CUDA FP16), извлечение 512-d CLIP эмбеддингов и слияние инвариантов по пространственно-семантическому критерию без дублирования сущностей.

### 2. RTR (Обратный ход — Развёртка и Порождение)
* **`MTV1` (Мотиватор):** Вычисляет вектор невязки потребности (дистанция до целевого объекта), стимулируя систему обнулить $b$-вектор.
* **`PRD1` (Семантический предиктор):** Генерирует цепочку токенов действий OPRN (схему `BLOM`) через авторегрессионную политику и поиск в долговременной памяти `TRL1Log` с расчетом точки подступа Stand-off ($0.5\text{ м}$ перед столом).
* **`SPL1` (Моторный мозжечок):** Рассчитывает непрерывные приращения скоростей $(\Delta v_x, \Delta w_z)$ для гладкого движения по дугам и производит локальный обход препятствий (**Elastic $b$-vector Warping**) без сброса глобальной цели.
* **Символический Щит Безопасности:** Детерминированный математический барьер raycast'а габаритного цилиндра шасси по вокселям MP10 перед отправкой команд на колеса.

---

## 🛠 Технологический Стек

* **C++20 Core (Высокопроизводительное ядро):**
  * `Eigen3`, `OctoMap`, `ZeroMQ (cppzmq)`, `Protocol Buffers 3`, `GoogleTest`.
  * Прямой доступ к памяти ($O(1)$) для графов сплайсов, raycasting $< 0.2\text{ мс}$.
* **Python 3.10+ (Машинное зрение и Симуляция):**
  * **PyBullet:** Физическая 3D-среда, моделирование контакта колес, рендеринг синтетической RGB-D камеры на реальном астрономическом времени (60 FPS).
  * **Ultralytics YOLOv8 + OpenAI CLIP ViT-B/32:** Высокоскоростной GPU-инференс детекции и семантики на **NVIDIA CUDA FP16** ($< 40\text{ мс}$ на кадр).
  * **ZeroMQ + WebSockets (asyncio):** Межпроцессный мост телеметрии и стриминга данных.
* **Web Frontend (Three.js WebGL):**
  * Браузерный 3D-инспектор пространства `MP10Space`, визуализатор сплайнов OPRN, габаритного цилиндра безопасности и HUD-панель телеметрии.

---

## 🚀 Быстрый Старт

### Системные требования
* **ОС:** Linux (Ubuntu 22.04 / 24.04 LTS или WSL2 на Windows).
* **Компилятор:** GCC $\ge 11$ или Clang $\ge 14$ с поддержкой C++20.
* **Среда сборки:** CMake $\ge 3.20$, Protobuf Compiler (`protoc`), Python $\ge 3.10$.
* **GPU (Опционально):** NVIDIA GPU с поддержкой CUDA для ускорения ML-инференса (автоматический fallback на CPU).

### 1. Единый автоматический запуск (Рекомендуемый)
Скрипт `run_demo.sh` автоматически компилирует C++ проект, генерирует Protobuf-биндинги, запускает сервисы стека и открывает стриминг:

```bash
# Запуск симуляции с Three.js веб-дашбордом (Headless физика)
./run_demo.sh

# Запуск с отображением нативного GUI окна симулятора PyBullet
./run_demo.sh --gui

# Запуск в режиме воспроизведения эталонного датасета TUM RGB-D
./run_demo.sh --dataset
```
После старта веб-интерфейс дашборда доступен в браузере: **`http://localhost:8000`**.

### 2. Ручная сборка C++ бинарников и тестирование
```bash
# 1. Генерация Protobuf для Python
protoc -I=02_data_models -I=04_interfaces --python_out=src/ml_service \
       02_data_models/gtr1_structures.proto 04_interfaces/ipc_messages.proto

# 2. Конфигурация и сборка Release бинарников C++
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
cmake --build . -j4

# 3. Запуск полного набора автоматизированных тестов CTest
ctest --output-on-failure
```
*Результат: **58/58 тестов успешно пройдены (100% pass rate)**.*

---

## 📂 Структура Репозитория

```
demonstration GTR1/
│
├── 📜 README.md                    # Главная точка входа, маппинг Спецификация ⇄ Код
├── 📜 description_GTR1.md          # Фундаментальный философско-технический манифест GTR1
├── 📜 issues.md                    # Центральный реестр задач (Backlog Issues 01-11)
├── 📜 run_demo.sh                  # Единый оркестратор запуска демонстрации
├── 📜 CMakeLists.txt               # Конфигурация сборки C++20 ядра
│
├── 📁 01_architecture/             # [СПЕЦИФИКАЦИЯ] Целевая архитектура и схемы
│   ├── TARGET_ARCHITECTURE.md      # Мастер-спецификация целевой системы (795 строк)
│   ├── target_cognitive_architecture.puml # Компонентная PlantUML архитектура
│   ├── cognitive_cycle_sequence.puml      # Sequence-диаграмма тактового цикла
│   ├── state_machine.puml                 # FSM жизненного цикла сплайсов OPRN/BLOM
│   ├── timing_dfd_package.puml            # DFD и временные бюджеты подсистем
│   ├── deployment.puml                    # Топология нативного развертывания и IPC
│   ├── ADR_001_MVP_baseline.md            # Архитектурные решения базового уровня
│   └── semantics_gtr.md                   # Семантические инварианты GTR1
│
├── 📁 02_data_models/              # [СПЕЦИФИКАЦИЯ] Модели данных и протоколы
│   ├── gtr1_structures.proto       # Protobuf схемы MP10, MP11, MP12, MP13, TRL1
│   ├── core_classes.puml           # UML-диаграмма доменных классов C++
│   └── data_formats.md             # Соглашения о системах координат и UUIDv4
│
├── 📁 04_interfaces/               # [СПЕЦИФИКАЦИЯ] Контракты межпроцессного обмена
│   └── ipc_messages.proto          # Protobuf схемы SensorFrame, DetectionRequest, Cmd
│
├── 📁 05_requirements/             # [СПЕЦИФИКАЦИЯ] Требования и ограничения
│   ├── Safety_and_Robotics.md      # Ограничения кинематики и аварийный E-Stop
│   ├── ML_and_SLAM_Specs.md        # Параметры OctoMap, пороговые значения CLIP/YOLO
│   └── NFR.md                      # Бюджеты задержек, памяти RAM/VRAM и таймауты
│
├── 📁 06_algorithms_ml/            # [СПЕЦИФИКАЦИЯ] Алгоритмические функции
│   └── A_star_cost_function.md     # Эвристическая функция стоимости планировщика
│
├── 📁 07_infrastructure/           # [ИНФРАСТРУКТУРА] Телеметрия, стримеры и Web UI
│   ├── dashboard/                  # Three.js WebGL интерфейс (index.html, app.js)
│   ├── sim_stream_receiver.cpp     # C++ ZeroMQ приемник кадров симулятора
│   ├── dataset_player.cpp          # C++ плеер эталонного датасета TUM RGB-D
│   └── datasets/                   # Эталонные RGB-D данные для регрессионных тестов
│
├── 📁 08_project_management/       # [УПРАВЛЕНИЕ] Дорожные карты и планы спринтов
│   ├── roadmap.md                  # Дорожная карта реализации проекта
│   └── dashboard_mp10.md           # Спецификация визуализатора пространства
│
├── 📁 issues/                      # [СПЕЦИФИКАЦИЯ ЗАДАЧ] Детальные спецификации Issues
│   ├── ISSUE-09_tickrate_sync.md   # Синхронизация тикрейта и TICK_BUNDLE (RESOLVED)
│   ├── ISSUE-10_action_token_predictor.md # Action-Token Transformer в PRD1 (OPEN)
│   └── ISSUE-11_dynamic_spl_kinematic_controller.md # Dynamic Splicer в SPL1 (OPEN)
│
├── 📁 src/                         # [КОД РЕАЛИЗАЦИИ] Исходный код системы
│   ├── main.cpp                    # Точка входа C++ супервизора когнитивного цикла
│   ├── core/                       # Доменная память: MP10, MP11, MP12, MP13, TRL1, фильтры
│   ├── operators/                  # Операторы: SLGWNode, Prd1Predictor, Spl1Splicer
│   ├── ml_service/                 # Python сервисы: cnn1_worker.py (CUDA), ws_bridge.py
│   └── simulation/                 # Python симуляция: pybullet_env.py (60 FPS)
│
├── 📁 tests/                       # [ТЕСТЫ] Модульные и интеграционные тесты CTest
│   ├── test_rtr_cycle.cpp          # Сквозное тестирование контура RTR
│   ├── test_slgw.cpp               # Тестирование шлюза SLGW и буфера Drop-Latest
│   ├── test_mp11_merge.cpp         # Пространственно-семантическое слияние
│   ├── test_pybullet_sim.cpp       # Верификация IPC сообщений симулятора
│   └── test_filters.cpp            # Фильтры Калмана SE(3) и карты глубин
│
└── 📁 reviews/                     # [АУДИТ] История спринтерских ревью и чекпоинтов
    ├── 20261009/                   # Первичное ревью архитектурного фундамента
    ├── 20261010/                   # Аудит когнитивного ядра и формирования бэклога
    └── 20261018/                   # Чекпоинт SDD, целевая архитектура и аудит
```

---

## 📖 Ссылки на Ключевые Документы

* **Сквозная целевая спецификация системы:** [01_architecture/TARGET_ARCHITECTURE.md](01_architecture/TARGET_ARCHITECTURE.md)
* **Реестр актуальных задач и бэклог:** [issues.md](issues.md)
* **Философско-технический базис GTR1:** [description_GTR1.md](description_GTR1.md)
* **Архитектурный контракт MVP (ADR 001):** [01_architecture/ADR_001_MVP_baseline.md](01_architecture/ADR_001_MVP_baseline.md)
* **Канонический онтологический глоссарий:** [glossary.md](glossary.md)
