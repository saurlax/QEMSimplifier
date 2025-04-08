# QEM Simplifier

A 3D model viewer with Quadric Error Metrics (QEM) mesh simplification capabilities. This application allows users to load 3D models, view them with customizable camera controls, and reduce their polygon count using the QEM algorithm while preserving model features.

## Usage

```bash
QEMReduction.exe <path_to_model>
```

Example:

```bash
QEMReduction.exe models/bunny.obj
```

## Controls

### Camera Controls:

- **Left Mouse Button + Drag**: Rotate camera around model
- **Right Mouse Button + Drag**: Pan camera
- **Mouse Wheel**: Zoom in/out
- **W/A/S/D Keys**: Move camera forward/left/backward/right

### Mesh Simplification:

- **Minus Key (-)**: Reduce model detail (decrease face count)
- **Plus/Equal Key (+)**: Increase model detail (increase face count)
- **E Key**: Export simplified model to a file (creates a .min.obj file)
