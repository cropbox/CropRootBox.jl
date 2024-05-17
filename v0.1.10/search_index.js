var documenterSearchIndex = {"docs":
[{"location":"#CropRootBox.jl","page":"Home","title":"CropRootBox.jl","text":"","category":"section"},{"location":"","page":"Home","title":"Home","text":"CropRootBox.jl implements a root system architecture simulation algorithm described in CRootBox model. Our implementation is written in a domain-specific language based on Julia using Cropbox framework. While Cropbox framework was primarily designed for helping development of conventional process-based crop models with less dynamic structural development in mind, it is still capable of handling complex structure as envisioned by functional-structural plant models (FSPM).","category":"page"},{"location":"#Installation","page":"Home","title":"Installation","text":"","category":"section"},{"location":"","page":"Home","title":"Home","text":"using Pkg\nPkg.add(\"CropRootBox\")","category":"page"},{"location":"#Getting-Started","page":"Home","title":"Getting Started","text":"","category":"section"},{"location":"","page":"Home","title":"Home","text":"ENV[\"UNITFUL_FANCY_EXPONENTS\"] = true","category":"page"},{"location":"","page":"Home","title":"Home","text":"using Cropbox\nusing CropRootBox","category":"page"},{"location":"","page":"Home","title":"Home","text":"BaseRoot is a common system inherited by other systems representing each root type, i.e., PrimaryRoot, FirstOrderLateralRoot, and SecondOrderLateralRoot in this example.","category":"page"},{"location":"","page":"Home","title":"Home","text":"parameters(CropRootBox.BaseRoot; alias = true)","category":"page"},{"location":"","page":"Home","title":"Home","text":"Here is an example configuration for simulating maize root growth. Some parameter values are annotated with standard deviation after ± indicating actual values are randomly sampled from normal distribution as needed.","category":"page"},{"location":"","page":"Home","title":"Home","text":"config = @config(\n    :RootArchitecture => :maxB => 5,\n    :BaseRoot => :T => [\n        # P F S\n          0 1 0 ; # P\n          0 0 1 ; # F\n          0 0 0 ; # S\n    ],\n    :PrimaryRoot => (;\n        lb = 0.1 ± 0.01,\n        la = 18.0 ± 1.8,\n        ln = 0.6 ± 0.06,\n        lmax = 89.7 ± 7.4,\n        r = 6.0 ± 0.6,\n        Δx = 0.5,\n        σ = 10,\n        θ = 80 ± 8,\n        N = 1.5,\n        a = 0.04 ± 0.004,\n        color = CropRootBox.RGBA(1, 0, 0, 1),\n    ),\n    :FirstOrderLateralRoot => (;\n        lb = 0.2 ± 0.04,\n        la = 0.4 ± 0.04,\n        ln = 0.4 ± 0.03,\n        lmax = 0.6 ± 1.6,\n        r = 2.0 ± 0.2,\n        Δx = 0.1,\n        σ = 20,\n        θ = 70 ± 15,\n        N = 1,\n        a = 0.03 ± 0.003,\n        color = CropRootBox.RGBA(0, 1, 0, 1),\n    ),\n    :SecondOrderLateralRoot => (;\n        lb = 0,\n        la = 0.4 ± 0.02,\n        ln = 0,\n        lmax = 0.4,\n        r = 2.0 ± 0.2,\n        Δx = 0.1,\n        σ = 20,\n        θ = 70 ± 10,\n        N = 2,\n        a = 0.02 ± 0.002,\n        color = CropRootBox.RGBA(0, 0, 1, 1),\n    )\n)\n; # hide","category":"page"},{"location":"","page":"Home","title":"Home","text":"An instance of RootArchitecture system is created with an instance of Pot describing a boundary of root growth.","category":"page"},{"location":"","page":"Home","title":"Home","text":"b = instance(CropRootBox.Pot)\ns = instance(CropRootBox.RootArchitecture; config, options = (; box = b), seed = 0)\nr = simulate!(s, stop = 100u\"d\")\n; # hide","category":"page"},{"location":"","page":"Home","title":"Home","text":"After 100 days of simulation, we can draw a rendering of root structure bounded in the pot.","category":"page"},{"location":"","page":"Home","title":"Home","text":"using GLMakie\nscn = CropRootBox.render(s)\nGLMakie.save(\"root_maize.png\", scn; size = (1600, 3200))\n; # hide","category":"page"},{"location":"","page":"Home","title":"Home","text":"(Image: )","category":"page"},{"location":"","page":"Home","title":"Home","text":"For more information about using the framework such as instance() and simulate() functions, please refer to the Cropbox documentation.","category":"page"}]
}