
```markdown name=src/robot_controllers/README.md
```markdown
# robot_controllers

Controller‑Konfigurationen für Mecanum & DiffDrive (YAMLs), plus evtl. Anpassungen / Parameter‑Libraries.

Wichtige Dateien
- config/mecanum_drive_controller.yaml
- config/diff_drive_controller.yaml
- config/joint_state_broadcaster.yaml
- examples/ — example controller spawn commands

Hinweis
- Stelle sicher, dass die `wheel_names` / `left_wheel` / `right_wheel` in diesen YAMLs exakt mit URDF & hardware interface übereinstimmen.
- Falls du das ros2_controllers mecanum Paket verwendest, befolge dessen Parameter‑library (generate_parameter_library) — param‑dateien sollten vollständig dokumentiert sein.

Test
- Simuliere cmd_vel und prüfe, ob die wheel command topics/command interfaces befüllt werden.