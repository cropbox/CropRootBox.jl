using Cropbox
using CropRootBox
using Test

root_maize = @config(
    :RootArchitecture => :maxB => 5,
    :BaseRoot => :T => [
        # P F S
          0 1 0 ; # P
          0 0 1 ; # F
          0 0 0 ; # S
    ],
    :PrimaryRoot => (;
        lb = 0.1 ± 0.01,
        la = 18.0 ± 1.8,
        ln = 0.6 ± 0.06,
        lmax = 89.7 ± 7.4,
        r = 6.0 ± 0.6,
        Δx = 0.5,
        σ = 10,
        θ = 80 ± 8,
        N = 1.5,
        a = 0.04 ± 0.004,
        color = CropRootBox.RGBA(1, 0, 0, 1),
    ),
    :FirstOrderLateralRoot => (;
        lb = 0.2 ± 0.04,
        la = 0.4 ± 0.04,
        ln = 0.4 ± 0.03,
        lmax = 0.6 ± 1.6,
        r = 2.0 ± 0.2,
        Δx = 0.1,
        σ = 20,
        θ = 70 ± 15,
        N = 1,
        a = 0.03 ± 0.003,
        color = CropRootBox.RGBA(0, 1, 0, 1),
    ),
    :SecondOrderLateralRoot => (;
        lb = 0,
        la = 0.4 ± 0.02,
        ln = 0,
        lmax = 0.4,
        r = 2.0 ± 0.2,
        Δx = 0.1,
        σ = 20,
        θ = 70 ± 10,
        N = 2,
        a = 0.02 ± 0.002,
        color = CropRootBox.RGBA(0, 0, 1, 1),
    )
)

@testset "root" begin
    b = instance(CropRootBox.Pot)
    s = instance(CropRootBox.RootArchitecture; config = root_maize, options = (; box = b), seed = 0)
    r = simulate!(s, stop = 100u"d")
    @test r.time[end] == 100u"d"
    # using GLMakie
    # scn = CropRootBox.render(s)
    # GLMakie.save("root_maize.png", scn)
    CropRootBox.writevtk(tempname(), s)
    # CropRootBox.writepvd(tempname(), CropRootBox.RootArchitecture, config = root_maize, stop = 50)
end
