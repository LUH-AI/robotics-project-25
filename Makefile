.PHONY: clean clean-build clean-pyc clean-test coverage dist docs help install check format
.DEFAULT_GOAL := help

define BROWSER_PYSCRIPT
import os, webbrowser, sys

from urllib.request import pathname2url

webbrowser.open("file://" + pathname2url(os.path.abspath(sys.argv[1])))
endef
export BROWSER_PYSCRIPT

define PRINT_HELP_PYSCRIPT
import re, sys

for line in sys.stdin:
	match = re.match(r'^([a-zA-Z_-]+):.*?## (.*)$$', line)
	if match:
		target, help = match.groups()
		print("%-20s %s" % (target, help))
endef
export PRINT_HELP_PYSCRIPT

PYTHON ?= python
CYTHON ?= cython
PYTEST ?= uv run pytest
CTAGS ?= ctags
PIP ?= uv pip
MAKE ?= make
PRECOMMIT ?= uv run pre-commit
RUFF ?= uv run ruff
MYPY ?= uv run mypy
ISORT ?= uv run isort
BROWSER := python -c "$$BROWSER_PYSCRIPT"

help:
	@python -c "$$PRINT_HELP_PYSCRIPT" < $(MAKEFILE_LIST)

clean: clean-build clean-pyc ## remove all build, test, coverage and Python artifacts

clean-build: ## remove build artifacts
	rm -fr build/
	rm -fr dist/
	rm -fr .eggs/
	find . -name '*.egg-info' -exec rm -fr {} +
	find . -name '*.egg' -exec rm -f {} +

clean-pyc: ## remove Python file artifacts
	find . -name '*.pyc' -exec rm -f {} +
	find . -name '*.pyo' -exec rm -f {} +
	find . -name '*~' -exec rm -f {} +
	find . -name '__pycache__' -exec rm -fr {} +

clean-test: ## remove test and coverage artifacts
	rm -fr .tox/
	rm -f .coverage
	rm -fr htmlcov/
	rm -fr .pytest_cache
ruff: ## run ruff as a formatter
	$(RUFF) check --fix --silent --exit-zero robotics_project_25
	$(RUFF) check --fix --exit-zero --no-cache robotics_project_25
isort:
	$(ISORT) robotics_project_25 tests
test: ## run tests quickly with the default Python
	python setup.py tests

coverage: ## check code coverage quickly with the default Python
	coverage run --source robotics_project_25 setup.py test
	coverage report -m
	coverage html
	$(BROWSER) htmlcov/index.html
docs: ## generate Sphinx HTML documentation, including API docs
	rm -f docs/robotics_project_25.rst
	rm -f docs/modules.rst
	sphinx-apidoc -o docs/ robotics_project_25
	$(MAKE) -C docs clean
	$(MAKE) -C docs html
	$(BROWSER) docs/_build/html/index.html
install: clean ## install the package to the active Python's site-packages
	$(PIP) install -e ".[dev]"

pre-commit:
	$(PRECOMMIT) run --all-files

format:
	make ruff
	make isort