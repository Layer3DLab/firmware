#!/bin/sh
usage() {
	echo "usage: tag <vX.X> or tag <vX.X.X>"
}

if [ $# -ne 1 ]; then
	usage
	exit 1
fi

REPO_DIR="$(PWD)"
filepath="$REPO_DIR/firmware_src/language.h"

if [ -f "$filepath" ]; then
    sed -i "" "s/#define GIT_TAG \"[^\"]\"/#define GIT_TAG \"$1\"/g" "$filepath"
    git commit -am "Bumped version number to $1"
    git tag "$1"
    exit 0
else
    echo "$filepath does not exist"
    exit 1
fi