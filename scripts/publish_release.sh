#!/usr/bin/env bash
# Publishes the build in dist/ (firmware-*.bin and version.txt) as a GitHub
# release. Usage: publish_release.sh channel|archive
#
#   channel - the branch's rolling OTA channel, "<branch>-latest". Devices
#     poll its fixed asset URLs, so an existing release is updated in place
#     rather than recreated (which would 404 them for the length of a publish):
#     GitHub can't replace an asset atomically, so each file is uploaded under
#     a temporary name and swapped in (delete old, rename new). Binaries go
#     before version.txt, so a device polling midway reads the old version and
#     leaves the new binaries alone. Only the default branch's channel gets
#     GitHub's "Latest" badge.
#
#   archive - one untouched release per build, "<branch>-<UTC commit
#     date>-<sha>": the record of firmware production devices may have run.
#     The date makes tags sort chronologically on the releases page (which
#     orders by name), below the rolling "-latest" tag.
#
# cleanup.yml parses both tag formats — keep it in sync.
#
# Runs in GitHub Actions: needs GH_TOKEN, GITHUB_REPOSITORY, GITHUB_REF_NAME
# and GITHUB_SHA, plus VERSION (the build's version) and, for channel,
# DEFAULT_BRANCH.
set -euo pipefail

repo=$GITHUB_REPOSITORY
branch=${GITHUB_REF_NAME//\//-}  # tags can't contain slashes
files=(dist/*.bin dist/version.txt)
built="commit $VERSION"  # the version is the short commit SHA

# create TAG TITLE NOTES LATEST
create() {
  gh release create "$1" "${files[@]}" --repo "$repo" --target "$GITHUB_SHA" \
    --title "$2" --notes "$3" --latest="$4"
}

archive() {
  local stamp tag
  stamp=$(TZ=UTC git show -s --format=%cd --date=format-local:%Y%m%d-%H%M%S "$GITHUB_SHA")
  tag="$branch-$stamp-$VERSION"
  # A re-run for the same commit replaces its own archive.
  gh release delete "$tag" --yes --cleanup-tag --repo "$repo" || true
  create "$tag" "$tag (build history)" \
    "Archived build for branch \`$GITHUB_REF_NAME\` — $built. Not an OTA channel; devices don't poll this." \
    false
}

channel() {
  local tag="$branch-latest" title notes latest=false
  title="$tag (OTA channel)"
  notes="Rolling OTA channel for branch \`$GITHUB_REF_NAME\` — devices poll this fixed URL; CI overwrites it on every push. Built from $built."
  [ "$GITHUB_REF_NAME" = "$DEFAULT_BRANCH" ] && latest=true

  if ! gh release view "$tag" --repo "$repo" >/dev/null 2>&1; then
    create "$tag" "$title" "$notes" "$latest"  # first build: nothing polls it yet
    return
  fi

  # Stage every file as "<name>.new" (--clobber clears leftovers of an earlier
  # run that failed midway).
  local staging f
  staging=$(mktemp -d)
  for f in "${files[@]}"; do cp "$f" "$staging/$(basename "$f").new"; done
  gh release upload "$tag" "$staging"/* --clobber --repo "$repo"

  # One listing gives every asset id needed for the swaps.
  local -A ids
  local name id
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

  # Point the tag and the release's recorded target at this commit, and
  # refresh the text. The binaries are already live, so a refused tag move
  # only warns.
  gh api -X PATCH "repos/$repo/git/refs/tags/$tag" -f sha="$GITHUB_SHA" -F force=true \
    >/dev/null || echo "::warning::could not move tag $tag to $GITHUB_SHA"
  gh release edit "$tag" --repo "$repo" --target "$GITHUB_SHA" \
    --title "$title" --notes "$notes" --latest="$latest"
}

case "${1:-}" in
  channel) channel ;;
  archive) archive ;;
  *) echo "usage: $0 channel|archive" >&2; exit 2 ;;
esac
