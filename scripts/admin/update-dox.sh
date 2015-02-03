#!/bin/sh

# Thanks: http://stackoverflow.com/questions/14710257/running-a-cron-job-at-230-am-every-day
# On how to automate process at 2:30 every day (type "date" to get your server time)
# crontab -e
# 30 2 * * * /your/command

echo "Update teo-main..."
cd $HOME/teo-main
svn up --username wikirobots
echo "Doxy teo-main..."
cd teo-main/doc
rm -r html
/usr/local/bin/doxygen
cd ../..

echo "Doxy teo-body..."
cd teo-body/doc
rm -r html
/usr/local/bin/doxygen
cd ../..

echo "Doxy teo-head..."
cd teo-head/doc
rm -r html
/usr/local/bin/doxygen
cd ../..

