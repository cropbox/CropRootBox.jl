module CropRootBox

using Cropbox
using Distributions
import Makie
import Meshing
using GeometryBasics: GeometryBasics, Mesh, Point3f0
using CoordinateTransformations: IdentityTransformation, LinearMap, Transformation, Translation
using Rotations: RotZX
using Colors: RGBA
import UUIDs

@system Rendering

@system Container(Rendering, Controller) begin
    dist(; p::Point3f0): distance => -Inf ~ call
end

@system Pot(Container) <: Container begin
    r1: top_radius => 10 ~ preserve(u"cm", parameter)
    r2: bottom_radius => 6 ~ preserve(u"cm", parameter)
    h: height => 30 ~ preserve(u"cm", parameter)
    sq: square => false ~ preserve::Bool(parameter)

    dist(nounit(r1), nounit(r2), nounit(h), sq; p::Point3f0): distance => begin
        x, y, z = p
        if z < -h # below
            -z - h
        elseif 0 < z # above
            z
        else # inside: -h <= z <= 0
            w = -z / h # [0, 1]
            r = (1-w)*r1 + w*r2
            if sq
                max(abs(x), abs(y)) - r
            else
                sqrt(x^2 + y^2) - r
            end
        end
    end ~ call
end

mesh(s::Pot) = begin
    r1 = Cropbox.deunitfy(s.r1', u"cm")
    r2 = Cropbox.deunitfy(s.r2', u"cm")
    h = Cropbox.deunitfy(s.h', u"cm")
    GeometryBasics.Mesh(x -> s.dist'(x), GeometryBasics.Rect(GeometryBasics.Vec(-2r1, -2r2, -1.5h), GeometryBasics.Vec(4r1, 4r2, 3h)), Meshing.MarchingCubes(), samples=(50, 50, 50))
end

@system Rhizobox(Container) <: Container begin
    l: length => 16u"inch" ~ preserve(u"cm", parameter)
    w: width => 10.5u"inch" ~ preserve(u"cm", parameter)
    h: height => 42u"inch" ~ preserve(u"cm", parameter)

    dist(nounit(l), nounit(w), nounit(h); p::Point3f0): distance => begin
        x, y, z = p
        if z < -h # below
            -z - h
        elseif 0 < z # above
            z
        else # inside: -h <= z <= 0
            d = abs(y) - w/2
            d < 0 ? abs(x) - l/2 : d
        end
    end ~ call
end

mesh(s::Rhizobox) = begin
    l = Cropbox.deunitfy(s.l', u"cm")
    w = Cropbox.deunitfy(s.w', u"cm")
    h = Cropbox.deunitfy(s.h', u"cm")
    g = GeometryBasics.Rect3D(Point3f0(-l/2, -w/2, 0), Point3f0(l, w, -h))
    GeometryBasics.mesh(g)
end

@system SoilCore(Container) <: Container begin
    d: diameter => 5 ~ preserve(u"cm", parameter)
    l: length => 90 ~ preserve(u"cm", parameter)
    x0: x_origin => 0 ~ preserve(u"cm", parameter)
    y0: y_origin => 0 ~ preserve(u"cm", parameter)

    dist(nounit(d), nounit(l), nounit(x0), nounit(y0); p::Point3f0): distance => begin
        x, y, z = p
        if z < -l # below
            -z - l
        elseif 0 < z # above
            z
        else # inside: -l <= z <= 0
            sqrt((x - x0)^2 + (y - y0)^2) - d/2
        end
    end ~ call
end

mesh(s::SoilCore) = begin
    d = Cropbox.deunitfy(s.d', u"cm")
    l = Cropbox.deunitfy(s.l', u"cm")
    x0 = Cropbox.deunitfy(s.x0', u"cm")
    y0 = Cropbox.deunitfy(s.y0', u"cm")
    g = GeometryBasics.Cylinder(Point3f0(x0, y0, 0), Point3f0(x0, y0, -l), Float32(d)/2)
    GeometryBasics.mesh(g)
end

@system SoilLayer(Container) <: Container begin
    d: depth => 0 ~ preserve(u"cm", parameter)
    t: thickness => 10 ~ preserve(u"cm", parameter)

    dist(nounit(d), nounit(t); p::Point3f0): distance => begin
        x, y, z = p
        a = -d
        b = a - t
        if a <= z # above
            z - a
        elseif z < b # below
            -(z - b)
        else # inside: b <= z < a
            z
        end
    end ~ call
end

@system Tropism begin
    N: tropsim_trials => 1.0 ~ preserve(parameter)
    to(; α, β): tropism_objective => 0 ~ call
end

@system Plagiotropism(Tropism) <: Tropism begin
    RT0: parent_transformation ~ hold
    to(RT0; α, β): tropism_objective => begin
        R = RotZX(β, α) |> LinearMap
        (RT0 ∘ R).linear[9] |> abs
    end ~ call
end

@system Gravitropism(Tropism) <: Tropism begin
    RT0: parent_transformation ~ hold
    to(RT0; α, β): tropism_objective => begin
        R = RotZX(β, α) |> LinearMap
        #-(RT0 ∘ R).linear[9]
        p = (RT0 ∘ R)(Point3f0(0, 0, -1))
        p[3]
    end ~ call
end

@system Exotropism(Tropism) <: Tropism begin
    to(; α, β): tropism_objective => begin
        #HACK: not exact implementation, needs to keep initial heading
        abs(Cropbox.deunitfy(α))
    end ~ call
end

@system RootSegment{
    Segment => RootSegment,
    Branch => RootSegment,
}(Tropism, Rendering) begin
    box ~ <:Container(override)

    ro: root_order => 1 ~ preserve::int(extern)
    zi: zone_index => 0 ~ preserve::int(extern)

    zt(lmax, la, lb, lp, ln): zone_type => begin
        if (lmax - la) <= lp
            :apical
        elseif lp < lb
            :basal
        else
            #HACK: assume apical when no lateral branching zone exists
            iszero(ln) ? :apical : :lateral
        end
    end ~ preserve::sym

    lb: length_of_basal_zone => 0.4 ~ preserve(u"cm", extern, parameter, min=0)
    la: length_of_apical_zone => 0.5 ~ preserve(u"cm", extern, parameter, min=0)
    ln: length_between_lateral_branches => 0.3 ~ preserve(u"cm", extern, parameter, min=0)
    lmax: maximal_length => 3.9 ~ preserve(u"cm", extern, parameter, min=0.1)

    r: maximum_elongation_rate => 1.0 ~ preserve(u"cm/d", extern, parameter, min=0)
    pr(r): potential_elongation_rate ~ track(u"cm/d")

    t(context.clock.time): timestamp ~ preserve(u"hr")
    Δl(Δx) ~ preserve(u"cm", max=lr)
    lp: parent_length => 0 ~ preserve(u"cm", extern)
    ls: sibling_length => 0 ~ preserve(u"cm", extern)
    l(pr): length ~ accumulate(u"cm", max=Δl, when=!im)
    lr(lp, lmax): remaining_length => lmax - lp ~ track(u"cm")
    lt(lp, l): total_length => lp + l ~ track(u"cm")
    li(ls, l): interleaving_length => ls + l ~ track(u"cm")
    ls1(li): next_sibling_length ~ track(u"cm", when=!ib)

    Δx: axial_resolution => 1 ~ preserve(u"cm", parameter)
    σ: standard_deviation_of_angle => 30 ~ preserve(u"°", parameter)
    σ_Δx(σ, nounit(Δx)): normalized_standard_deviation_of_angle => sqrt(Δx)*σ ~ preserve(u"°")

    θ: insertion_angle => 30 ~ preserve(u"°", parameter)
    pα(zi, nounit(θ), nounit(σ_Δx);): pick_angular_angle => begin
        θ = zi == 0 ? θ : zero(θ)
        rand(Normal(θ, σ_Δx))
    end ~ call(u"°")
    pβ(;): pick_radial_angle => rand(Uniform(0, 360)) ~ call(u"°")
    αN: angular_angle_trials => 20 ~ preserve::int(parameter)
    βN: radial_angle_trials => 5 ~ preserve::int(parameter)
    A(pα, pβ, to, N, dist=box.dist, np, αN, βN): angles => begin
        n = rand() < N % 1 ? ceil(N) : floor(N)
        P = [(pα(), pβ()) for i in 0:n]
        O = [to(α, β) for (α, β) in P]
        (o, i) = findmin(O)
        (α, β) = P[i]
        d = dist(np(α, β))
        for i in 1:αN
            #HACK: look around 360° rather than original 90° in case the segment already is out of boundary
            α1 = α + 360u"°" * (i-1)/αN
            for j in 1:βN
                d < 0 && break
                β1 = pβ()
                d1 = dist(np(α1, β1))
                if d1 < d
                    d = d1
                    α, β = α1, β1
                end
            end
            d < 0 && break
        end
        (α, β)
    end ~ preserve::Tuple
    α(A): angular_angle => A[1] ~ preserve(u"°")
    β(A): radial_angle => A[2] ~ preserve(u"°")

    RT0: parent_transformation ~ track::Transformation(override)
    pp(RT0): parent_position => RT0(Point3f0(0, 0, 0)) ~ preserve::Point3f0
    np(RT0, nounit(Δl); α, β): new_position => begin
        R = RotZX(β, α) |> LinearMap
        (RT0 ∘ R)(Point3f0(0, 0, -Δl))
    end ~ call::Point3f0
    RT(nounit(l), α, β): local_transformation => begin
        # put root segment at parent's end
        T = Translation(0, 0, -l)
        # rotate root segment
        R = RotZX(β, α) |> LinearMap
        R ∘ T
    end ~ track::Transformation
    RT1(RT0, RT): global_transformation => RT0 ∘ RT ~ track::Transformation
    cp(RT1): current_position => RT1(Point3f0(0, 0, 0)) ~ track::Point3f0

    a: radius => 0.05 ~ preserve(u"cm", extern, parameter, min=0.01)

    c: color => RGBA(1, 1, 1, 1) ~ preserve::RGBA(parameter)

    n: name ~ hold
    T: transition ~ hold
    nb(T, name;): next_branch => begin
        find(r) = begin
            d = T[name]
            for (k, v) in d
                r < v ? (return k) : (r -= v)
            end
            :nothing
        end
        find(rand())
    end ~ call::sym

    ms(l, Δl, lt, lmax): may_segment => (l >= Δl && lt < lmax) ~ flag
    S(n, box, ro, zi, r, lb, la, ln, lmax, lt, ls1, wrap(RT1), a): segment => begin
        #HACK: keep lb/la/ln/lmax parameters same for consecutive segments
        produce(eval(n); box, ro, zi=zi+1, r, lb, la, ln, lmax, lp=lt, ls=ls1, RT0=RT1, a)
    end ~ produce::Segment(when=ms)

    mb(zt, li, ln): may_branch => (zt == :lateral && li >= ln) ~ flag
    B(nb, box, ro, wrap(RT1)): branch => begin
        #HACK: eval() for Symbol-based instantiation based on tabulate-d matrix
        produce(eval(nb()); box, ro=ro+1, RT0=RT1)
    end ~ produce::Branch(when=mb)

    ii(cp; c::Container): is_inside => (c.dist'(cp) <= 0) ~ call::Bool
    im(l, Δl, lt, lmax): is_mature => (l >= Δl || lt >= lmax) ~ flag

    is(S): is_segmented => !isnothing(S) ~ flag
    ib(B): is_branched => !isnothing(B) ~ flag
end

mesh(s::RootSegment) = begin
    l = Cropbox.deunitfy(s.l', u"cm")
    a = Cropbox.deunitfy(s.a', u"cm")
    (iszero(l) || iszero(a)) && return nothing
    r = a/2
    g = GeometryBasics.Rect3D(Point3f0(-r, -r, 0), Point3f0(a, a, l))
    m = GeometryBasics.mesh(g)

    #HACK: reconstruct a mesh with transformation applied
    mv = GeometryBasics.decompose(GeometryBasics.Point, m)
    mf = GeometryBasics.decompose(GeometryBasics.TriangleFace{Int}, m)
    M = s.RT1'
    m = GeometryBasics.normal_mesh(M.(mv), mf)

    c = s.color'
    n = length(GeometryBasics.coordinates(m))
    GeometryBasics.pointmeta(m; color=fill(c, n))
end

#TODO: provide @macro / function to automatically build a series of related Systems
@system BaseRoot(RootSegment) <: RootSegment begin
    T: transition ~ tabulate(rows=(:PrimaryRoot, :FirstOrderLateralRoot, :SecondOrderLateralRoot, :ThirdOrderLateralRoot), parameter)
end
@system ThirdOrderLateralRoot{
    Segment => ThirdOrderLateralRoot,
}(BaseRoot, Gravitropism) <: BaseRoot begin
    n: name => :ThirdOrderLateralRoot ~ preserve::sym
end
@system SecondOrderLateralRoot{
    Segment => SecondOrderLateralRoot,
    Branch => ThirdOrderLateralRoot,
}(BaseRoot, Gravitropism) <: BaseRoot begin
    n: name => :SecondOrderLateralRoot ~ preserve::sym
end
@system FirstOrderLateralRoot{
    Segment => FirstOrderLateralRoot,
    Branch => SecondOrderLateralRoot,
}(BaseRoot, Gravitropism) <: BaseRoot begin
    n: name => :FirstOrderLateralRoot ~ preserve::sym
end
@system PrimaryRoot{
    Segment => PrimaryRoot,
    Branch => FirstOrderLateralRoot,
}(BaseRoot, Gravitropism) <: BaseRoot begin
    n: name => :PrimaryRoot ~ preserve::sym
end

Cropbox.update!(s::BaseRoot, t) = begin
    #HACK: merely checking im flag would miss pre-stage update for producing S
    if s.is'
        update!(s.S', t)
        update!(s.B', t)
    else
        Cropbox._update!(s, t)
    end
end

@system RootArchitecture(Controller) begin
    box(context) ~ <:Container(override)
    minB: minimum_number_of_basal_roots => 1 ~ preserve(parameter)
    maxB: number_of_basal_roots => 1 ~ preserve(parameter, min=minB)
    RT0: initial_transformation => IdentityTransformation() ~ track::Transformation
    roots(roots, box, maxB, wrap(RT0)) => begin
        [produce(PrimaryRoot; box, RT0) for i in (length(roots)+1):maxB]
    end ~ produce::PrimaryRoot[]
end

render(s::RootArchitecture; soilcore=nothing, resolution=(500, 500)) = begin
    #HACK: comfortable default size when using WGLMakie inside Jupyter Notebook
    scene = Makie.Scene(; resolution)
    Makie.mesh!(scene, mesh(s))
    #HACK: customization for container
    Makie.mesh!(scene, mesh(s.box), color=(:black, 0.02), transparency=true, shading=false)
    !isnothing(soilcore) && Makie.mesh!(scene, mesh(soilcore), color=(:purple, 0.1), transparency=true, shading=false)
    scene
end

mesh(s::RootArchitecture; container=nothing) = begin
    meshes = GeometryBasics.Mesh[]
    gather!(s, Rendering; store=meshes, callback=render!, kwargs=(; container))
    isempty(meshes) ? nothing : merge(meshes)
end
render!(g::Gather, r::RootSegment, ::Val{:Rendering}; container=nothing) = begin
    m = isnothing(container) || r.ii'(container) ? mesh(r) : nothing
    !isnothing(m) && push!(g, m)
    visit!(g, r; container)
end
render!(g::Gather, a...; k...) = visit!(g, a...; k...)

gatherbaseroot!(g::Gather, s::BaseRoot, ::Val{:BaseRoot}) = (push!(g, s); visit!(g, s))
gatherbaseroot!(g::Gather, a...) = visit!(g, a...)

gathergeom!(g::Gather,  r::BaseRoot, ::Val{:BaseRoot}) = begin
    r.zi' == 0 && push!(g, (
        point=[r.pp', r.cp', r.S["**"].cp'...],
        radius=[r.a', r.a', r.S["**"].a'...] |> Cropbox.deunitfy,
        timestamp=[r.t', r.t', r.S["**"].t'...] |> Cropbox.deunitfy,
    ))
    visit!(g, r)
end
gathergeom!(g::Gather, a...) = visit!(g, a...)

using WriteVTK
gathervtk(name::AbstractString, s::System) = begin
    L = gather!(s, BaseRoot; callback=gathergeom!)
    P = Float32[]
    C = MeshCell[]
    i = 0
    for l in L
        lp = l.point
        [append!(P, p) for p in lp]
        n = length(lp)
        I = collect(1:n) .+ i
        i += n
        c = MeshCell(VTKCellTypes.VTK_POLY_LINE, I)
        push!(C, c)
    end
    P3 = reshape(P, 3, :)
    g = vtk_grid(name, P3, C)
    for k in (:radius, :timestamp)
        D = [l[k] for l in L] |> Iterators.flatten |> collect
        isempty(D) && continue
        g[string(k), VTKPointData()] = D
    end
    g
end
writevtk(name::AbstractString, s::System) = vtk_save(gathervtk(name, s))
writepvd(name::AbstractString, S::Type{<:System}; kwargs...) = begin
    pvd = paraview_collection(name)
    path = mkpath("$name-pvd")
    i = 0
    simulate(S; kwargs...) do D, s
        pvd[i] = gathervtk("$path/$name-$i", s)
        i += 1
    end
    vtk_save(pvd)
end

using MeshIO
using FileIO
writestl(name::AbstractString, s::System) = writestl(name, mesh(s))
writestl(name::AbstractString, m::Mesh) = save(File{format"STL_BINARY"}(name), m)
writestl(name::AbstractString, ::Nothing) = @warn "no mesh available for writing $name"

end
