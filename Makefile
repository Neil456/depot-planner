PYTHON ?= python3
CPP_BUILD_DIR ?= build/cpp
CMAKE ?= cmake
CTEST ?= ctest
CLANG_FORMAT ?= clang-format

.PHONY: setup test step1 battery equivalence gifs report all clean \
        cpp-configure cpp-build cpp-test cpp-bench format format-check

setup:
	pip install -e ".[dev]"

test:
	$(PYTHON) -m pytest -q

step1:
	$(PYTHON) scripts/demo_grid.py

# BACKEND selects the planner implementation and ENGINE the closed-loop runner;
# both default to the C++ core and the Python loop respectively.
battery:
	$(PYTHON) scripts/run_battery.py $(if $(BACKEND),--backend $(BACKEND)) $(if $(ENGINE),--engine $(ENGINE))
	$(PYTHON) scripts/run_parking_battery.py $(if $(BACKEND),--backend $(BACKEND))

equivalence:
	$(PYTHON) scripts/check_equivalence.py

gifs:
	$(PYTHON) scripts/demo_scenarios.py
	$(PYTHON) scripts/demo_parking.py
	$(PYTHON) scripts/make_gifs.py
	$(PYTHON) scripts/make_parking_gifs.py
	$(PYTHON) scripts/make_showcase.py

report:
	$(PYTHON) scripts/bench_cpp.py
	$(PYTHON) scripts/make_report.py
	$(PYTHON) scripts/make_readme.py

all: setup test step1 battery gifs report

# ------------------------------------------------------------------------ C++

# A standalone build of the C++ core with the GoogleTest and Google Benchmark
# targets enabled. `pip install -e .` builds only the library and the extension,
# so an install never needs to fetch the test dependencies.
cpp-configure:
	$(CMAKE) -S cpp -B $(CPP_BUILD_DIR) -DCMAKE_BUILD_TYPE=$(if $(CPP_BUILD_TYPE),$(CPP_BUILD_TYPE),Release) \
	  -DDEPOT_BUILD_PYTHON=OFF -DDEPOT_BUILD_TESTS=ON -DDEPOT_BUILD_BENCH=ON $(CMAKE_EXTRA)

cpp-build: cpp-configure
	$(CMAKE) --build $(CPP_BUILD_DIR) --parallel

cpp-test: cpp-build
	$(CTEST) --test-dir $(CPP_BUILD_DIR) --output-on-failure

cpp-bench: cpp-build
	$(CPP_BUILD_DIR)/bench/depot_bench --benchmark_min_time=0.5s

format:
	$(CLANG_FORMAT) -i $(shell find cpp -name '*.cpp' -o -name '*.hpp')

format-check:
	$(CLANG_FORMAT) --dry-run --Werror $(shell find cpp -name '*.cpp' -o -name '*.hpp')

clean:
	rm -rf results/step1 results/step2 results/step5 results/gifs results/*.csv
	rm -rf results/README_assets/report
	rm -rf $(CPP_BUILD_DIR)
