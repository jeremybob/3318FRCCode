# Dashboard Windows Packaging

The custom dashboard can now be packaged as a standalone Windows application with a bundled Java runtime.

## Outputs

- `./gradlew.bat packageDashboardWindows`
  Produces a self-contained Windows app image with `3318 Dashboard.exe`.
- `./gradlew.bat packageDashboardWindowsInstaller`
  Produces an installable Windows `.exe` wrapper around the same packaged app.

Both outputs are written under:

- `build/dashboard-package/windows/image`
- `build/dashboard-package/windows/installer`

## Requirements

- Run packaging on a Windows machine.
- Use a full JDK 17, not a JRE.
- `packageDashboardWindowsInstaller` also requires WiX on the packaging host because `jpackage` uses it to build the installer executable.

## Behavior

- The packaged launcher bundles its own Java runtime.
- Desktop WPILib/vendor native libraries are staged into the package automatically.
- The launcher defaults to `--team 3318`.

## Recommended Workflow

1. On a Windows build machine, open `cmd` in the repo root.
2. Run:

```bat
gradlew.bat packageDashboardWindows
```

3. Copy the generated `3318 Dashboard` folder to the Driver Station laptop.
4. Launch `3318 Dashboard.exe`.

For a one-click packaging flow, you can also run:

```bat
package-dashboard-windows.bat
```

If you want an installer instead of copying the folder, run:

```bat
gradlew.bat packageDashboardWindowsInstaller
```

Or use the helper:

```bat
package-dashboard-windows.bat installer
```

## Version Override

The package version defaults to `2026.1.0`.

Override it with:

```bat
gradlew.bat packageDashboardWindows -PdashboardPackageVersion=2026.3.6
```
