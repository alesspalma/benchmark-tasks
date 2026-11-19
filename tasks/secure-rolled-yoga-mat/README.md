# Secure a rolled yoga mat in a two-loop carrying sling

This task prototype models a full-size exercise mat roll and an elastic-loop mat carrier with no buckles. In the physical workflow, the mat is rolled by hand, each elastic loop goes over an end, and the user lifts the mat by the connecting strap. The springy outer turn has to stay inside the carrier while the load is off the work surface. The mat sizing and carrier mechanism follow these references.

- [Hugger Mugger mat sizing guide](https://www.huggermugger.com/pages/yoga-mat-guide)
- [Hugger Mugger carrying strap method](https://www.huggermugger.com/blogs/yoga-gear/how-to-use-a-yoga-mat-strap-for-easy-transport-and-support)
- [Sunshine Yoga elastic end-loop sling](https://www.sunshineyoga.com/elastic-carrying-straps-for-yoga-mats/)
- [Beemat elasticated loop carrier](https://www.beemat.co.uk/collections/mat-storage-and-accessories/products/yoga-equipment)

## Native prototype trial

Activate the workspace Isaac Lab runtime and run the world and reference with local-only device access. The trial defaults to seed 1415158438 and 640 actions at 20 Hz, which is 32 seconds of simulated control. Each output directory is created once. The receipt logs actual actions and source identity, along with the result measured by the scorer.

```sh
python -B tasks/secure-rolled-yoga-mat/run.py --output /tmp/yoga-mat-probe --device cuda:0 --physics-device cpu --no-cameras
```

The current native plan is a first-loop experiment. The full insertion sequence and shoulder carry are not yet validated.

## Asset provenance

| Asset | Source and license |
| --- | --- |
| Rolled exercise mat and thin nonslip support pad | Original generated USD primitives, MIT. |
| Elastic loops and connected shoulder webbing | Original generated USD primitives, MIT. |
| Exercise bench, YAM gantry, camera stands and lighting | yam-isaac-lab-arena, Apache 2.0. |
| YAM meshes and robot data | Runtime assets of yam-isaac-lab-arena, including the I2RT YAM model under MIT. |

Manufacturer links describe the physical use. No manufacturer meshes, photographs or other files are bundled.
