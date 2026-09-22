PYTHON ?= python3

.PHONY: setup test step1 battery gifs report all clean

setup:
	pip install -e ".[dev]"

test:
	$(PYTHON) -m pytest -q

step1:
	$(PYTHON) scripts/demo_grid.py

battery:
	$(PYTHON) scripts/run_battery.py
	$(PYTHON) scripts/run_parking_battery.py

gifs:
	$(PYTHON) scripts/make_gifs.py
	$(PYTHON) scripts/make_parking_gifs.py

report:
	$(PYTHON) scripts/make_report.py

all: setup test step1 battery gifs report

clean:
	rm -rf results/step1 results/gifs results/failures results/*.csv
