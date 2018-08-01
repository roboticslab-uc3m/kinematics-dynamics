#!/usr/bin/env bash

set +e

# see http://gronlier.fr/blog/2015/01/adding-code-coverage-to-your-c-project/

#-- Capture coverage info
lcov --directory . --capture --output-file coverage.info

#-- Filter out system and test code
lcov --remove coverage.info  '/usr/*' 'tests/*' --output-file coverage.info

#-- Debug before upload
lcov --list coverage.info

#-- Upload to coveralls
coveralls-lcov --source-encoding=ISO-8859-1 coverage.info
