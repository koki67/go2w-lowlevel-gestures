COMPOSE := docker compose
SERVICE := controller
PYTHON := /opt/venv/bin/python
SLOW_SCRIPT := /app/go2w_gesture_real.py
LEGACY_SCRIPT := /app/go2w_gesture_real_legacy.py

.PHONY: help build test clean describe \
	describe-slow describe-slow-height describe-slow-roll \
	describe-legacy describe-legacy-height describe-legacy-roll \
	describe-height describe-roll \
	preflight-slow-height preflight-slow-roll \
	preflight-legacy-height preflight-legacy-roll \
	preflight-height preflight-roll \
	live-slow-height live-slow-roll \
	live-legacy-height live-legacy-roll \
	live-height live-roll

help:
	@echo "Non-hardware: make build | test | describe"
	@echo "Slow 2.0/2.0 read-only: make preflight-slow-height | preflight-slow-roll"
	@echo "Slow 2.0/2.0 physical:  make live-slow-height | live-slow-roll"
	@echo "Legacy 1.0/0.5 read-only: make preflight-legacy-height | preflight-legacy-roll"
	@echo "Legacy 1.0/0.5 physical:  make live-legacy-height | live-legacy-roll"
	@echo "Compatibility: preflight-height/roll and live-height/roll use slow"

build:
	$(COMPOSE) build --pull

describe: describe-slow describe-legacy

describe-slow:
	$(COMPOSE) run --rm --no-deps -T --entrypoint $(PYTHON) \
		$(SERVICE) $(SLOW_SCRIPT) --describe

describe-slow-height:
	$(COMPOSE) run --rm --no-deps -T --entrypoint $(PYTHON) \
		$(SERVICE) $(SLOW_SCRIPT) --gesture height --describe

describe-slow-roll:
	$(COMPOSE) run --rm --no-deps -T --entrypoint $(PYTHON) \
		$(SERVICE) $(SLOW_SCRIPT) --gesture roll --describe

describe-legacy:
	$(COMPOSE) run --rm --no-deps -T --entrypoint $(PYTHON) \
		$(SERVICE) $(LEGACY_SCRIPT) --describe

describe-legacy-height:
	$(COMPOSE) run --rm --no-deps -T --entrypoint $(PYTHON) \
		$(SERVICE) $(LEGACY_SCRIPT) --gesture height --describe

describe-legacy-roll:
	$(COMPOSE) run --rm --no-deps -T --entrypoint $(PYTHON) \
		$(SERVICE) $(LEGACY_SCRIPT) --gesture roll --describe

describe-height: describe-slow-height

describe-roll: describe-slow-roll

test:
	$(COMPOSE) run --rm --no-deps -T \
		--entrypoint /opt/venv/bin/python $(SERVICE) \
		-m unittest discover -s /app/tests -v

preflight-slow-height:
	$(COMPOSE) run --rm --no-deps --entrypoint $(PYTHON) \
		$(SERVICE) $(SLOW_SCRIPT) --gesture height

preflight-slow-roll:
	$(COMPOSE) run --rm --no-deps --entrypoint $(PYTHON) \
		$(SERVICE) $(SLOW_SCRIPT) --gesture roll

preflight-legacy-height:
	$(COMPOSE) run --rm --no-deps --entrypoint $(PYTHON) \
		$(SERVICE) $(LEGACY_SCRIPT) --gesture height

preflight-legacy-roll:
	$(COMPOSE) run --rm --no-deps --entrypoint $(PYTHON) \
		$(SERVICE) $(LEGACY_SCRIPT) --gesture roll

preflight-height: preflight-slow-height

preflight-roll: preflight-slow-roll

live-slow-height:
	$(COMPOSE) run --rm --no-deps --entrypoint $(PYTHON) \
		$(SERVICE) $(SLOW_SCRIPT) --gesture height --live

live-slow-roll:
	$(COMPOSE) run --rm --no-deps --entrypoint $(PYTHON) \
		$(SERVICE) $(SLOW_SCRIPT) --gesture roll --live

live-legacy-height:
	$(COMPOSE) run --rm --no-deps --entrypoint $(PYTHON) \
		$(SERVICE) $(LEGACY_SCRIPT) --gesture height --live

live-legacy-roll:
	$(COMPOSE) run --rm --no-deps --entrypoint $(PYTHON) \
		$(SERVICE) $(LEGACY_SCRIPT) --gesture roll --live

live-height: live-slow-height

live-roll: live-slow-roll

clean:
	$(COMPOSE) down --remove-orphans
