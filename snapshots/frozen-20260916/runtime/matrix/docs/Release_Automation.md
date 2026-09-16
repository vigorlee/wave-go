# Release Automation

This workflow treats `jszr_mujoco_ue2` as the source project and `matrix` as the release/runtime project.

## Directory Layout

```text
/home/user/work/workspace/
  jszr_mujoco_ue2/   # Unreal Engine source project
  matrix/            # release/runtime project
```

## Full Pipeline

From the `matrix` directory:

```bash
bash scripts/release_manager/release_pipeline.sh 0.1.3 --all --upload --publish --make-latest
```

The pipeline runs:

1. `jszr_mujoco_ue2/Script/package_with_chunks.sh`
2. `jszr_mujoco_ue2/Script/package_for_release.sh`
3. Copy release tarballs into `matrix/releases`
4. `matrix/scripts/release_manager/package_assets.sh` to create `assets-VERSION.tar.gz`
5. Split files larger than GitHub Release's 2 GB asset limit
6. Regenerate SHA256 files and `manifest-VERSION.json`
7. Install local release artifacts into `matrix`
8. Run `scripts/check_env.sh runtime`
9. Upload/publish the GitHub Release when `--upload` or `--publish` is set

`assets-VERSION.tar.gz` is generated from the current runnable `matrix` tree. It
contains launcher binaries, MuJoCo runtime files, MC runtime/config/model files,
UE Engine runtime libraries, and dynamic map payloads. The assets packaging step
copies files into the tarball and does not delete runtime files from `matrix`.

## Common Commands

Package and test locally without uploading:

```bash
bash scripts/release_manager/release_pipeline.sh 0.1.3 --all
```

Package only selected chunks:

```bash
bash scripts/release_manager/release_pipeline.sh 0.1.3 --chunk-ids 0,1,24
```

Use a clean UE package build:

```bash
bash scripts/release_manager/release_pipeline.sh 0.1.3 --all --clean
```

Upload but keep the release as draft:

```bash
bash scripts/release_manager/release_pipeline.sh 0.1.3 --all --upload
```

Run only copy/manifest/test after UE packages already exist:

```bash
bash scripts/release_manager/release_pipeline.sh 0.1.3 --skip-ue-package --skip-release-package
```

## Notes

- `--publish` implies upload and publishes the release without an interactive prompt.
- `--upload` uploads artifacts but keeps a newly created release as draft.
- `--make-latest` updates GitHub's latest-release pointer after upload/publish.
- Use `--install-deps` only on machines where `sudo apt` dependency installation is expected.
- The matrix repository should use the GENISOM git identity for open-source `zsibot/*` commits.
