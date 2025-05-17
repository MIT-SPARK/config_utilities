# Config Utilities Dynamic Config GUI

# Installation

## Dependencies
Local:
```bash
pip install Flask
```

System:
```
sudo apt install python3-flask
```

# Known Limitations
Known limitations that we can (easily) fix in the future.

- [ ] Yaml(-cpp) float precision during encoding gives undersired behaviors, e.g.
  - Values and defaults have odd decimals
  - This results in float `selectOneOf` checks not working on the GUI side (which chcks for equality)
  - This breaks some utests in some cases (added `checkNear` tests for this)
- [ ] Currently assumes only one dynamic config client operates. 
  - As a result will not automatically update to changes in the ecosystem that the GUI did not cause itself.
  - Can easily be fixed by pressing refresh, bnut requires the user to do so
- [ ] In the future find better update logic than having to refresh the page every time, but didn't find a better solution.