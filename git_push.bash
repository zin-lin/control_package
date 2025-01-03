#!/bin/bash

git add .
git commit -m $1
git push origin $2

cd ../..
git add .
git commit -m "vehicle control changes"
git push origin master

