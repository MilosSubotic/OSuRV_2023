
module Units

	export
		units_table,
		p,
		n,
		μ,
		u,
		m,
		k,
		K,
		meg,
		M,
		G,
		NumberUnit,
		convert,
		show,
		parse,
		parse_item
		
	###########################################################################
		
	@assert VERSION >= v"1"

	using ArgParse
	
	###########################################################################
	
	units_table = Dict{String, Float64}(
		"p" => 1e-12,
		"n" => 1e-9,
		"μ" => 1e-6,
		"u" => 1e-6,
		"m" => 1e-3,
		"" => 1,
		"k" => 1e3,
		"K" => 1e3,
		"meg" => 1e6,
		"M" => 1e6,
		"G" => 1e9,
	)
	
	#TODO Generate these constants and export them from units_table.
	const p = 1e-12
	const n = 1e-9
	const μ = 1e-6
	const u = μ
	const m = 1e-3
	const k = 1e3
	const K = 1e3
	const meg = 1e6
	const M = 1e6
	const G = 1e9
	
	
	struct NumberUnit <: Number
		number::Float64
		unit::String
	end
	
	function NumberUnit(n::Number)
		if n >= 1e9
			return NumberUnit(n*1e-9, "G")
		elseif n >= 1e6
			return NumberUnit(n*1e-6, "M")
		elseif n >= 1e3
			return NumberUnit(n*1e-3, "K")
		elseif n >= 1
			return NumberUnit(n, "")
		elseif n >= 1e-3
			return NumberUnit(n*1e3, "m")
		elseif n >= 1e-6
			return NumberUnit(n*1e6, "μ")
		elseif n >= 1e-9
			return NumberUnit(n*1e9, "n")
		else
			return NumberUnit(n*1e12, "p")
		end
	end
	
	function Base.convert(::Type{T}, nu::NumberUnit) where T <: Number
		return T(nu.number*units_table[nu.unit])
	end
	
	function Base.show(io::IO, nu::NumberUnit)
		i, f = divrem(nu.number, 1)
		i = Int(i)
		if f == 0
			f = ""
		else
			f = abs(f)
			f = Float32(f)
			f = "$f"[3:end]
		end
		u = nu.unit
		if u == ""
			if f != ""
				u = "."
			else
				u = ""
			end
		end
		print(io, "$i$u$f")
	end
	function Base.show(io::IO, ::MIME"text/plain", nu::NumberUnit)
		show(io, nu)
		d = nu.number*units_table[nu.unit]
		print(io, "($d)")
	end
	
	function Base.parse(::Type{NumberUnit}, s::AbstractString)
		m = match(r"^([-+]?[0-9]*\.?[0-9]+([eE][-+]?[0-9]+)?)$", s)
		if m != nothing
			return NumberUnit(parse(Float64, m[1]))
		end
		m = match(r"^([-+]?[0-9]*\.?[0-9]+)$", s)
		if m != nothing
			return NumberUnit(parse(Float64, m[1]))
		end
		m = match(r"^([-+]?[0-9]*\.?[0-9]+)([A-Za-z]{1,3})$", s)
		if m != nothing
			u = m[2]
			if !haskey(units_table, u)
				msg(FATAL, "Cannot parse unit \"$u\"!")
			end
			n = parse(Float64, m[1])
			return NumberUnit(n*units_table[u])
		end
		m = match(r"^([-+]?[0-9]+)([A-Za-z]{1,3})([0-9]+)$", s)
		if m != nothing
			u = m[2]
			if !haskey(units_table, u)
				msg(FATAL, "Cannot parse unit \"$u\"!")
			end
			i = parse(Int64, m[1])
			f = parse(Float64, "0."*m[3])
			n = i + f
			return NumberUnit(n*units_table[u])
		end
		
		msg(FATAL, "Cannot parse NumberUnit \"$s\"!")
	end
	
	function ArgParse.parse_item(::Type{NumberUnit}, s::AbstractString)
		return parse(NumberUnit, s)
	end
	
	###########################################################################

end
