# Right Short Return + Slow Shooting

Rebuilt from branch `turret-camera-offset-fix`, source revision `a60abbd`.

The final trench crossing now uses y = 0.55 m at x = 4.30746 m and x = 6.11434 m, moving 5.2–13.3 cm closer to the low-Y wall. The later turn is unchanged. At the authored zero heading and editor robot width of 0.99 m, nominal bumper clearance is 5.5 cm; verify actual robot dimensions and tracking clearance before running. The stopScoring split point is recomputed along the adjusted approach, retaining its source progress ratio and the same speed limits.

The live **Right Single Swipe** auto loads `right-center-line-swipe`, `right-short-return-shoot`, and `right-shoot-exit` in order. Autos.java references match these renamed files.

Extract `short-return-depot-bline.zip`. In BLine Web, choose **Project → Import / Export → Import Autos Folder…**, select the extracted `autos` folder, and open **Right Short Return + Slow Shooting**.

The robot chooser in Autos.java includes that same routine. The deployed JSON files and this bundle use the current branch's actual `left-center-line-depot.json`, `depot-2.json`, and `depot-3.json` sources. Original autos, source paths, and global defaults are unchanged.

The route remains reflected across y = 4.035 m, using BLine's 8.07 m field width and negated headings. The return keeps only 25% of the final depot approach, ending at (2.26148, 2.29988) m. The exit begins at that same pose, and its next target is moved inward to y = 1.20 m. It does not wait for the depot wall sensor.

All constraints come from the current branch, with only moving-shot maximum velocity reduced to 0.30 m/s between startShooting and stopScoring. In particular:

- Collection retains 4.0 m/s at ordinals 0–2, 1.5 m/s at 3–4, and 2.0 m/s at 5.
- Return retains 2.0 m/s transit and 0.55 m/s before startShooting.
- Exit restores 0.50 m/s after stopScoring and the original defaults on later segments.
- The exit's added 2.0 m/s² initial acceleration constraint is retained.

Translation-only targets split the shooting boundaries without adding rotation targets. Event ratios and constraint ordinal ranges are remapped to these segments. All current source event keys and geometric positions on the shortened route are preserved, including Unwind and outtakeEvent; the removed old depotFullAgitateStart event is not reintroduced. Linked-target metadata is omitted so the old depot editing target cannot restore the removed wall approach.

Regenerate with `node tools/build-short-return-depot.cjs`. Checks verify every non-shooting constraint against its current source, all event positions, JSON read-back, unchanged collection mirror, and return/exit pose continuity. Gradual acceleration/deceleration still applies; robot timing and clearance require on-robot verification.
