#!/usr/bin/env bash

set -e

#-- Register SSH deploy key for AMOR API private repository
#-- https://gist.github.com/lukewpatterson/4242707#gistcomment-2382443
#-- http://anil.recoil.org/2013/10/06/travis-secure-ssh-integration.html

mkdir -p ~/.ssh

echo $DEPLOY_KEY_AMOR_API | base64 --decode | openssl aes-256-cbc -K $encrypted_9c9995b5c6b2_key -iv $encrypted_9c9995b5c6b2_iv -out ~/.ssh/id_rsa -d

chmod 600 ~/.ssh/id_rsa

echo "Host *" >> ~/.ssh/config
echo "    StrictHostKeyChecking no" >> ~/.ssh/config
echo "    CheckHostIP no" >> ~/.ssh/config
echo "    PasswordAuthentication no" >> ~/.ssh/config
echo "    UserKnownHostsFile=/dev/null" >> ~/.ssh/config
