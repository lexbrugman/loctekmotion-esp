#!/usr/bin/env bash
# Publishes the build in dist/ (firmware-*.bin and version.txt) as the branch's
# rolling OTA channel release, "<branch>-latest". Devices poll its fixed asset
# URLs, so an existing release is updated in place rather than recreated
# (which would 404 them for the length of a publish): GitHub can't replace an
# asset atomically, so each file is uploaded under a temporary name and
# swapped in (delete old, rename new). Binaries go before version.txt, so a
# device polling midway reads the old version and leaves the new binaries
# alone. Only the default branch's channel gets GitHub's "Latest" badge.
#
# Runs in GitHub Actions: needs GH_TOKEN, GITHUB_REPOSITORY, GITHUB_REF_NAME
# and GITHUB_SHA, plus VERSION (the build's version) and DEFAULT_BRANCH.
set -euo pipefail

repo=$GITHUB_REPOSITORY
tag="${GITHUB_REF_NAME//\//-}-latest"
title="$tag (OTA channel)"
notes="Rolling OTA channel for branch \`$GITHUB_REF_NAME\` — devices poll this fixed URL; CI overwrites it on every push. Built from ${GITHUB_SHA:0:7} (version $VERSION)."
latest=false
[ "$GITHUB_REF_NAME" = "$DEFAULT_BRANCH" ] && latest=true
files=(dist/*.bin dist/version.txt)

if ! gh release view "$tag" --repo "$repo" >/dev/null 2>&1; then
  # First build of this branch: nothing is polling the channel yet.
  gh release create "$tag" "${files[@]}" --repo "$repo" --target "$GITHUB_SHA" \
    --title "$title" --notes "$notes" --latest="$latest"
  exit 0
fi

# Stage every file as "<name>.new" (--clobber clears leftovers of an earlier
# run that failed midway).
staging=$(mktemp -d)
for f in "${files[@]}"; do cp "$f" "$staging/$(basename "$f").new"; done
gh release upload "$tag" "$staging"/* --clobber --repo "$repo"

# One listing gives every asset id needed for the swaps.
declare -A ids
while IFS=$'\t' read -r name id; do ids[$name]=$id; done < <(
  gh api "repos/$repo/releases/tags/$tag" --jq '.assets[] | "\(.name)\t\(.id)"')

swap_in() {
  local name new
  name=$(basename "$1")
  new=${ids[$name.new]:-}
  [ -n "$new" ] || { echo "::error::staged asset $name.new not found"; exit 1; }
  if [ -n "${ids[$name]:-}" ]; then
    gh api -X DELETE "repos/$repo/releases/assets/${ids[$name]}"
  fi
  gh api -X PATCH "repos/$repo/releases/assets/$new" -f name="$name" >/dev/null
  echo "Published $name"
}
for f in dist/*.bin; do swap_in "$f"; done
swap_in dist/version.txt

# Point the tag and the release's recorded target at this commit, and refresh
# the text. The binaries are already live, so a refused tag move only warns.
gh api -X PATCH "repos/$repo/git/refs/tags/$tag" -f sha="$GITHUB_SHA" -F force=true \
  >/dev/null || echo "::warning::could not move tag $tag to $GITHUB_SHA"
gh release edit "$tag" --repo "$repo" --target "$GITHUB_SHA" \
  --title "$title" --notes "$notes" --latest="$latest"
