# Open the tear-strip mailer

Feasibility prototype. The task uses a reinforced starter tab and a progressive
front-panel tear to free a hinged corrugated mailer. The carton remains on the
bench with a paperback inside. This operation is described by the
[manufacturer’s corrugated mailer catalog](https://www.mondigroup.com/products-and-solutions/ecommerce/corrugated-mailers/).

The prototype has a free 252 × 165 × 50 mm corrugated mailer, a 232 mm tear path
in its front flap and a native paperback inside. Two native bodies represent
the carton and its hinged cover. A native grippable leading piece transmits
equal and opposite tension to the simulated tear front. The force threshold
and stiffness are estimates. The exposed paper tape is rendered from the
measured tab and tear-front locations.

With the configured Isaac Lab-Arena and YAM support, the native trial is

```bash
python run.py --device cuda:0 --physics-device cpu --output /tmp/mailer-trial
```

The trial uses seed 1415158438, a 20 Hz reference and a 240 Hz physics step.
Each native camera records 640 × 360 frames at 20 Hz. The receipt and trace
record the observed final score and opening, tab pose and tension sequence.
The nominal run limit is 480 actions.

The second complete native trial scored 0.0. Settling displaced the exposed tab
and advanced the modeled tear before arrival, so the actual grip and full
opening were unverified. This source is archived at its tested revision.
