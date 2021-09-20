# This script will delete all runs that are under a workflow marked as disabled_manually
# it requires de `gh` cli and `jq` to be installed

$org='clubcapra'
$repo='capra_web_ui'

$workflow_ids=($(gh api repos/$org/$repo/actions/workflows | jq '.workflows | .[] | select(.state==\"disabled_manually\") | .id'))

foreach ($workflow_id in $workflow_ids) {
  write-host 'Deleting runs for workflow' $workflow_id
  $run_ids=( $(gh api repos/$org/$repo/actions/workflows/$workflow_id/runs | jq '.workflow_runs[].id') )
  foreach ($run_id in $run_ids) {
    gh api repos/$org/$repo/actions/runs/$run_id -X DELETE
  }
  write-host 'runs deleted'
}
