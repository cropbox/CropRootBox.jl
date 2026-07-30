using Cropbox
using CropRootBox
using Test

root_maize = @config(
    :RootArchitecture => :maxB => 5,
    :BaseRoot => :T => [
        # P F S    T
          0 1 0 #= 0 =#; # P
          0 0 1 #= 0 =#; # F
          0 0 0 #= 0 =#; # S
       #= 0 0 0    0   ; # T =#
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
    @testset "pot mesh" begin
        geometry = CropRootBox.GeometryBasics
        pot_mesh = CropRootBox.mesh(b)
        points = geometry.coordinates(pot_mesh)

        @test !isempty(points)
        @test !isempty(geometry.faces(pot_mesh))
        @test all(isfinite, Iterators.flatten(points))

        xmin, xmax = extrema(getindex.(points, 1))
        ymin, ymax = extrema(getindex.(points, 2))
        zmin, zmax = extrema(getindex.(points, 3))

        @test isapprox(xmin, -10; atol=1)
        @test isapprox(xmax, 10; atol=1)
        @test isapprox(ymin, -10; atol=1)
        @test isapprox(ymax, 10; atol=1)
        @test isapprox(zmin, -30; atol=1)
        @test isapprox(zmax, 0; atol=1)
    end

    s = instance(CropRootBox.RootArchitecture; config = root_maize, options = (; box = b), seed = 0)
    r = simulate!(s, stop = 100u"d")
    @test r.time[end] == 100u"d"

    geometry = CropRootBox.GeometryBasics
    root = first(s.roots')
    root_mesh = CropRootBox.mesh(root)
    attributes = geometry.vertex_attributes(root_mesh)

    @testset "root mesh attributes" begin
        @test haskey(attributes, :color)
        @test length(attributes.color) == length(geometry.coordinates(root_mesh))
        @test all(==(root.color'), attributes.color)
    end

    @testset "render" begin
        figure = CropRootBox.render(s; size=(200, 200))
        @test figure isa CropRootBox.Makie.Figure
    end

    CropRootBox.writevtk(tempname(), s)
    # CropRootBox.writepvd(tempname(), CropRootBox.RootArchitecture, config = root_maize, stop = 50)

    @testset "STL export" begin
        mktempdir() do directory
            path = joinpath(directory, "root.stl")
            CropRootBox.writestl(path, root)
            loaded = CropRootBox.FileIO.load(path)

            @test isfile(path)
            @test length(geometry.faces(loaded)) == length(geometry.faces(root_mesh))
            @test Set(geometry.coordinates(loaded)) == Set(geometry.coordinates(root_mesh))
        end
    end
end
