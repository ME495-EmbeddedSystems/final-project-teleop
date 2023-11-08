# Teleoperation Final Project

#### Group Members:
- Rohan Kota
- Luke Batteas
- Zach Alves
- Aditya Nair
- Leo Chen

#### Packages:
- `teleop`
- `teleop_tasks`
- `teleop_haptics`
- `teleop_sensing`
- `teleop_avatar`
- `teleop_interfaces`

#### Git Instructions:
Only edit your package while on the `feature/{package_name}` branch. You can switch to this branch using:
```bash
git switch feature/{package_name}
```

Before editing, make sure you pull the changes that have been pushed to main:
```bash
git rebase main
```

After you are done editing and testing your code, switch to the main branch and merge the code from whatever branch you were working on:
```bash
git switch main
git merge feature/{package_name}
```