# Release artifacts

[中文](../../zh/guides/releases.md) · [Documentation index](../README.md)

## Targets aligned with transport

The Hand release workflow uses the transport v0.3.4 target matrix:

- Ubuntu 20.04, 22.04, 24.04 and 26.04, each with x86_64 and aarch64 (ARM64) Release TGZ SDKs: eight archives.
- Windows x64 and ARM64, each with Debug `/MDd` and Release `/MD` MSVC v143 ZIP SDKs: four archives.

Linux builds run in native architecture containers for each distribution, recording the image digest,
using its default GCC, libstdc++ ABI1 and C++17. Windows builds use native runners and v143 14.44.
Container checks do not certify native kernels or hardware; ARM64 SDK validation does not establish
availability of proprietary ARM64 drivers.

Hand SDK packages contain both hand and paired transport static libraries, headers, CMake packages,
documentation and read-only command-line examples. Test executables and proprietary drivers/libraries
are excluded. Versions remain independent. Hand 0.3.8 requires transport 0.3.4 or newer in
the 0.3 minor line in source, prebuilt and installed-consumer builds.


The paired source pin and prebuilt downloads use the same v0.3.4 release.
Rebuild hand and downstream consumers with matching transport headers and libraries;
public transport structures have grown. No new physical hardware acceptance is claimed. Transport archives now use the `linkerbot-transport-` prefix.
Hand continues to consume the supported `linkerbot-comm` CMake package and CAN target aliases.

## Names and checksums

Examples use hand version `0.3.0`; compiler versions reflect the actual build:

```text
linkerbot-hand-cpp-0.3.0-Linux-aarch64-ubuntu24.04-GNU13.3.0-libstdcxx-abi1-Release.tar.gz
linkerbot-hand-cpp-0.3.0-Windows-ARM64-msvc143-19.44.35229.0-Debug-MDd.zip
```

Each archive has an adjacent `.sha256` file. The complete set also contains `release-manifest.json`
and `SHA256SUMS.txt`: 26 files in total. The manifest records both source SHAs, build identities,
archive sizes and verified object counts. Collection requires all twelve distinct targets and checks
filenames, metadata and every static-library object's actual machine type. Missing archives, checksum
mismatches, wrong architectures/CRT settings and source identity mismatches fail collection.

Changes to build/release files in pull requests, version tags or manual dispatch trigger validation
and upload Actions artifacts. PR artifacts describe the tested merge commit.

To attach the complete set to a GitHub Release, manually run **Release packages** on an existing
matching `v<version>` tag, supply the full transport SHA and version, and enable `create_draft`.
The exact tagged hand commit must already have a successful main-push or manually dispatched
**C++ Report Card** run under the A+ gate. All twelve builds and collection must pass first.
The optional job creates a new **draft** with all 26 files and downloads them to verify their bytes.
It rejects branch dispatches, moved/mismatched tags and existing releases; it never replaces assets.
Default runs only upload Actions artifacts. Publishing the draft remains a separate maintainer action.
Refer to actual attachments on [Releases](https://github.com/linker-bot/linkerbot-hand-cpp-sdk/releases)
for download availability. For separate prebuilt transport consumption, see the [build guide](validation.md).
