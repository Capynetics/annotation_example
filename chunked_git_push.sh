#!/bin/bash

echo "Script is running..."

folders=$(git status --porcelain | awk '/^\?\?/ && $2 ~ /_annotated\/$/ {print $2}')

if [ -z "$folders" ]; then
  echo "No untracked folders found."
  exit 0
fi

for folder in $folders; do
  echo "Processing: $folder"
  if [ -d "$folder" ] && [ "$(ls -A "$folder")" ]; then
    git add "$folder"
    git commit -m "Add $folder"
    git push
    echo "✅ Committed and pushed: $folder"
  else
    echo "⚠️ Skipping empty or non-existent folder: $folder"
  fi
done

echo "✅ All done."
