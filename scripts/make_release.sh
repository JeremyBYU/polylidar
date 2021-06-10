#!/bin/bash
# exit when any command fails
set -e

# keep track of the last executed command
trap 'last_command=$current_command; current_command=$BASH_COMMAND' DEBUG
# echo an error message before exiting
trap 'echo "\"${last_command}\" command filed with exit code $?."' EXIT

python scripts/manage_versions.py --bump patch
pip install . --use-feature=in-tree-build
cd src_docs && python make_docs.py && cd ..
git status

read -p "Are you sure you want to add all these files? " -n 1 -r
echo    # (optional) move to a new line
if [[ $REPLY =~ ^[Yy]$ ]]
then
    read -p "Enter commit message: " commmit_message
    git add .
    git commit -m "$commmit_message"
    python scripts/manage_versions.py --tag
    echo -e "Dont forget to push:\n \t git push origin dev --tags"
fi