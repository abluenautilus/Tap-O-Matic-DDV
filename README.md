# Tap-O-Matic DDV

The Olivia Artz Modular Time Machine, version by Harry Richardson, modified by Blue Nautilus and the DivKid Discord, and finally tweaked heavily by Eris Fairbanks. We've come full circle folks!

This version modifies the [Tap-O-Matic version](https://github.com/cormallen/tap-o-matic) by HR in two ways:
<ul>
<li>It moves the pan pots down by 2mm. This makes the build much easier since the pan pots don't butt up against the sliders.</li>
<li>It moves the power header on the back so that it doesn't interfere with one of the jacks. This means you don't have to shave plastic off the jack.</li>
</ul>

**Eris's Software Notes:**
- Read heads are paired in order to utilize the PSRAM cache more effectively.
- The dynamics management system is drastically improved, utilizing a 4-point lookahead system per read-head pair.
- Filters used to be one-poles, now they're my own SVF implementation. Steeper cutoff, more resonance, more character.
- Code is a bit more readable and cleaner I think, constants and twiddle factors gathered up and commented.
- CPU usage is better, though extreme time variation might still cause frame drops? I haven't seen any, but that doesn't mean they aren't possible. Might be good to bump block size up to 8 if we notice any.
- Feedback can still distort. Dynamic range still isn't what I'd like it to be and probably never will be unless there's a hardware change to support 20vpp.

## Front Panel

All of the files for producing the panels are in the panel folder. This panel was fabricated by http://pcbway.com. It's a PCB panel using matte black solder mask:

![image](pictures/front_panel.png)

## BOM

Our modified BOM is here:
https://docs.google.com/spreadsheets/d/1hPb3Es_wUxIoNBYNVZt8cKr9BfJMghtDLR-cnE-fVQ0/edit?gid=118381065#gid=118381065

## Fabrication Instructions

The fabrication guide can be found here: 

https://docs.google.com/document/d/1D_RPgzVUW2ujZSJS9yKFKmFV2mPIwQp9BS7iKvg6IVI/edit?tab=t.0


## Build Instructions

The build guide can be found here: 

https://docs.google.com/document/d/1rPKsOXEx5abdQNxCypnp2nViSWlIubEysEng-fZd7tw/edit?tab=t.0

