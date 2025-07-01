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

- [ ] Currently vectors and maps of configs don't show up if they're empty, would be great to have these to be able to add more entires from the GUI.
- [ ] Paramter verification on the GUI side (although implemented) does not quite seem to work.
- [ ] YAML parsing errors on the GUI side break the successfully parsed yaml values (why???)

### Feature Ideas:

- [ ] Optionally add better support for vectors and maps in the GUI, not too bad though right now.
- [ ] Add pure YAML interface as alternative option (gui settings).
- [ ] Could config save/loads
- [ ] Figure out desired behavior for changing virtual configs and adding new configs, i.e. should it automatically update? CUrrently wait for submit click if the user decides differently.