#!/bin/sh

# Thanks: http://stackoverflow.com/questions/14710257/running-a-cron-job-at-230-am-every-day
# On how to automate process at 2:30 every day (type "date" to get your server time)
# crontab -e
# 30 2 * * * /your/command

proj="kinematics-dynamics"
src="$HOME/$proj"
build="$src/doc/build"

echo "Update $proj..."
git -C "$src" pull

echo "Doxy $proj..."
mkdir -p "$build"
cmake -H"$src/doc" -B"$build" && make -C "$build" clean && make -C "$build" dox
