# Rule-Band API   ― lightweight sub-goal generator ―

## 📚 What is this?

A feather-weight, **rule-based** companion to the original *BandNet* Neural-RL
hierarchy. It ingests a *raw* scenario JSON (100×100 occupancy map +
global paths) and spits out a **single sub-goal coordinate** for your local
navigation stack – no GPU, no checkpoints, near-instant inference.

Key design choices  🗝️

* **Temperature-controlled softmax rule** – distance-to-goal (τ ∈ [0,1])
  expands / shrinks exploration bands deterministically.
* **Identical data pipeline** – shares the same map & band generation code as
  the neural version ⇒ drop-in replacement.
* **Built-in visual debug** – one flag gives you a colour-coded band map and
  red “✕” on the sampled cell for sanity checks.

---

## 📂 Project tree

```

RuleBand_API/
├─ rule_api.py              # main wrapper (RuleBandAPI class)
├─ example.py               # 10-line demo script
├─ requirements.txt         # minimal deps
│
├─ utils/
│   ├─ dataloading.py       # raw JSON ➜ tensors / band_map / τ
│   └─ environment_functions.py
│ 
└─ data/
    └─ example_data.json    # tiny toy scenario

````

---

## ⚡️ Quick-start

```bash
git clone git@github.com:ZhangJingru-Ruby/RuleBand_API.git
cd RuleBand_API

pip install -r requirements.txt

python example_rule.py --debug   # 🎨  pops heat-map + prints coordinate
````

Output:

```
Rule sub-goal: (2.13, -1.47)
```

A 100×100 plot appears – colour = band index, red ✕ = sampled cell.

---

## 🔍 How it works (under the hood)

| Stage         | Code path                                            | What happens                                                             |
| ------------- | ---------------------------------------------------- | ------------------------------------------------------------------------ |
| **Load**      | `utils.data_loader.preprocess_for_rule()`            | JSON → occupancy, paths, start/goal                                      |
| **Band map**  | `build_valid_mask_and_bands()`                       | distance-transform to global path, 10 concentric “safety-corridor” bands |
| **τ calc**    | `preprocess_for_rule()`                              | clip(dist_to_goal / 25 m, 0,1)                                         |
| **Prob rule** | `rule_api.rule_probs_temp()`                         | temperature-softmax: τ=0 ⇒ near bands, τ=1 ⇒ far bands                   |
| **Sample**    | `_sample_cell_from_band()`                           | pick random valid cell in chosen band, fallback if empty                 |
| **Convert**   | `map_to_world_coords()`                              | (mx,my) pixel back to metres                                             |
| **Debug viz** | `utils.environment_functions.visualize_band_probs()` | heat-map + probability bar (optional)                                    |

---

## 🎨 Visual debug

```python
x, y = api.predict_from_file("data/example_data.json", debug=True)
```

![viz](docs/viz_example.png)

* White  = invalid cells,
* 0‥9   = band index,
* Red ✕ = sampled sub-goal.

---

## 📣 Citation

If this code supports a publication, cite it!

```bibtex
@software{zhang_2025_ruleband,
  author = {Zhang, Jingru},
  title  = {RuleBand API: Lightweight Sub-Goal Generator for Hierarchical RL},
  year   = {2025},
  url    = {https://github.com/ZhangJingru-Ruby/RuleBand_API}
}
```
