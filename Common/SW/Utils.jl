
module Utils

	# Helpers.
	export
		@export_enum,
		@must_specify,
		msg,
		@display,
		prompt,
		prompt_answers,
		install_if_not_installed
	
	# Constants.
	export
		c0
	
	# Iterations.
	export
		progress,
		repeat_elems,
		range_rand,
		unzip,
		linspace2d
	
	# Signal stuff.
	export
		spectrum,
		dft,
		dft_spectrum,
		wrap,
		mix,
		linear_interpolation
	
	# Numeric stuff.
	export
		clamp,
		clamp!,
		clamp_neg,
		round,
		ceil,
		bool
	
	# Filesystem stuff.
	export
		glob,
		check_file,
		gen_nick_names
		
	###########################################################################
		
	@assert VERSION >= v"1"
	
	###########################################################################
	
	using Statistics
	
	###########################################################################
	
	macro export_enum(enum)
		Expr(
			:block,
			Expr(
				:call,
				Expr(:., __module__, :(:eval)),
				Expr(:export, enum)
			),
			Expr(
				:for,
				Expr(:(=), :s, Expr(:call, :instances, esc(enum))),
				Expr(
					:call,
					Expr(:., __module__, :(:eval)),
					Expr(
						:call,
						:Expr,
						:(:export),
						Expr(:call, :Symbol, :s)
					)
				)
			)
		)
	end
	
	macro must_specify(var)
		quote
			throw(ArgumentError("must specify " * $(string(var))))
		end
	end
	
	###########################################################################
	
	struct ExitException <: Exception
		ret_code::Int
	end

	@enum MsgType VERB DEBUG INFO WARN ERROR FATAL
	@export_enum MsgType

	function msg(
		msg_type::MsgType,
		args...
	)
		if msg_type == VERB
			color = Base.text_colors[:white]
			msg_type_str = "verbose"
		elseif msg_type == DEBUG
			color = Base.text_colors[:light_green]
			msg_type_str = "debug"
		elseif msg_type == INFO
			color = Base.text_colors[:light_blue]
			msg_type_str = "info"
		elseif msg_type == WARN
			color = Base.text_colors[:light_yellow]
			msg_type_str = "warning"
		elseif msg_type == ERROR
			color = Base.text_colors[:light_red]
			msg_type_str = "error"
		elseif msg_type == FATAL
			color = Base.text_colors[:light_red]
			msg_type_str = "fatal"
		end

		#TODO print_with_color does not work.
		print(
			color,
			msg_type_str, ": ",
			args..., '\n',
			Base.color_normal
		)

		if msg_type == FATAL
			throw(ExitException(1))
		end
	end

	###########################################################################
	
	macro display(var)
		quote
			println($(string(var)), " =")
			display($(esc(var)))
			println()
		end
	end
	
	
	###########################################################################	
	
	function prompt(msg)::Char
		print(msg, ' ')
		cmd = `bash -c 'read -n 1 char; echo -n $char'`
		procs = open(cmd, "r", stdin)
		bytes = read(procs.out)
		success(procs) || pipeline_error(procs)
		str = String(bytes)
		if length(str) == 0
			c = '\n'
		else
			c = str[1]
		end
		println("")
		return c
	end
	
	function prompt_answers(msg, answers)
		while true
			c = prompt(msg)
			if c in answers
				return c
			end
		end
	end

	###########################################################################	
	
	import Pkg
	function install_if_not_installed(pkgs::Vector{String})
		deps = Pkg.dependencies()
		#all_pkgs = String[dep.name for dep in values(deps)]
		installed_pkgs = String[
			dep.name for dep in values(deps)
				if dep.is_direct_dep && dep.version !== nothing
		]
		for pkg in pkgs
			if !(pkg in installed_pkgs)
				println("Installing $pkg...")
				Pkg.add(pkg)
			end
		end
	end

	###########################################################################	
	
	const c0 = 299792458 # [m/s]
	

	###########################################################################

	struct Progress{I}
		itr::I
		msg::String
		N::Int
	end

	"""
		progress(iter, msg = "Done: ")

	An iterator which yield what `iter` would be,
	but track progress and print it on console output.

	# Examples
	```jldoctest
	julia> a = 1:4;

	julia>  for aa in progress(a)
				@show aa
			end
	Done: 0%
	1
	Done: 25%
	2
	Done: 50%
	3
	Done: 75%
	4
	```
	"""
	progress(iter, msg::String = "Done: ") = Progress(iter, msg, length(iter))

	Base.length(e::Progress) = length(e.itr)
	Base.size(e::Progress) = size(e.itr)
	function Base.iterate(p::Progress, state=(0,))
		i, rest = state[1], Base.tail(state)
		n = iterate(p.itr, rest...)
		if n === nothing 
			println(p.msg, 100, '%')
			return n
		else
			prev_prog = div(100*(i-1), p.N)
			prog = div(100*i, p.N)
			if prev_prog != prog
				println(p.msg, prog, '%')
			end
			return n[1], (i+1, n[2])
		end
	end

	Base.eltype(::Type{Progress{I}}) where {I} = Tuple{Int, eltype(I)}

	Base.IteratorSize(::Type{Progress{I}}) where {I} = IteratorSize(I)
	Base.IteratorEltype(::Type{Progress{I}}) where {I} = IteratorEltype(I)
	
	###########################################################################

	function repeat_elems(
		a::AbstractArray{T},
		n::Integer
	) where {
		T <: Number
	}
		kron(a, ones(T, n))
	end

	function range_rand(
		min::Union{T, Vector{T}},
		max::Union{T, Vector{T}},
		n::Integer
	) where {
		T <: AbstractFloat
	}
		rand(T, n).*(max - min) .+ min
	end

	function range_rand(
		min::Vector{T},
		max::Vector{T},
		n::Integer
	) where {
		T <: Integer
	}
		x = Vector{T}(n)
		for i in 1:n
			x[i] = rand(min[i]:max[i])
		end
		x
	end
	function range_rand(
		min::T,
		max::T,
		n::Integer
	) where {
		T <: Integer
	}
		rand(min:max, n)
	end

	function unzip(input::Array)
		s = size(input)
		types  = map(typeof, first(input))
		output = map(T->Array{T}(s), types)

		for i = 1:length(input)
		   @inbounds for (j, x) in enumerate(input[i])
			   (output[j])[i] = x
		   end
		end

		output
	end

	function linspace2d(start, stop, len=100)
		# TODO Optimize.
		axis = collect(linspace(start, stop, len))
		xs = kron(axis, ones(eltype(axis), 1, len))
		ys = xs'
		xys = map((x, y)-> [x, y], xs, ys)
		xys
	end
	
	###########################################################################
	
	function spectrum(x)
		fx = fft(x)
		a = abs(fx)
		N_2 = length(x) >> 1
		ha = (1.0/N_2) * a[1:N_2+1]
		ha
	end

	function dft(
		x::AbstractArray{T},
		F::AbstractArray{T} = linspace(0, 0.5, 64)
	) where {
		T <: Number
	}
		@assert all(0 .<= F .<= 0.5)
		kernel = e.^(-im*T(2π)*F)

		dft = similar(F, Complex{T})
		for Fi in 1:length(F)
			dft[Fi] = 0
		end

		for xi in 1:length(x)
			dft += kernel.^xi * x[xi]
		end

		dft
	end

	function dft_spectrum(x, F)
		dft_x = dft(x, F)
		a = abs(dft_x)
		a
	end

	# For unwrap(), use DSP package.

	function wrap(n::T)::T where {T <: Number}
		while n > π
			n -= 2π
		end
		while n < -π
			n += 2π
		end
		return n
	end

	mix(x, y, a) = x*(1-a) + y*a

	function linear_interpolation(
		src::AbstractArray{T},
		scale::Number
	) where {
		T <: Number
	}

		N_src = length(src)
		N_dst = round(Int, scale*N_src)
		ratio = (N_src - 1) / (N_dst - 1)

		dst = Vector{T}(N_dst)

		for idx_dst in 0:N_dst-1
			(weight_h, src_l) = modf(idx_dst*ratio)
			weight_l = 1 - weight_h
			src_h = src_l + 1
			idx_src_l = floor(Int, src_l)
			idx_src_h = min(ceil(Int, src_h), N_src-1)

			dst[idx_dst+1] =
				src[idx_src_l+1]*weight_l + src[idx_src_h+1]*weight_h
		end

		dst
	end

	###########################################################################
	
	import Base: clamp, clamp!
	function clamp(
		x::Vector,
		lo::Vector,
		hi::Vector,
	)
		y = copy(x)
		@assert length(y) == length(lo)
		@assert length(y) == length(hi)
		for i in 1:length(y)
			@assert lo[i] <= hi[i]
			if y[i] < lo[i]
				y[i] = lo[i]
			elseif y[i] > hi[i]
				y[i] = hi[i]
			end
		end
		y
	end
	function clamp!(
		x::AbstractArray,
		lo::AbstractArray,
		hi::AbstractArray,
	)
		@assert length(x) == length(lo)
		@assert length(x) == length(hi)
		for i in 1:length(x)
			@assert lo[i] <= hi[i]
			if x[i] < lo[i]
				x[i] = lo[i]
			elseif x[i] > hi[i]
				x[i] = hi[i]
			end
		end
	end
	clamp_neg(x) = max(x, 0)

	import Base: round, ceil
	round(t::Type{T}, f::AbstractFloat) where {T <: AbstractFloat} = T(f)
	ceil(t::Type{T}, f::AbstractFloat) where {T <: AbstractFloat} = T(f)
	
	bool(x::Int) = x != 0
	bool(x::Nothing) = false
	bool(x::UnitRange{Int}) = x != 0:-1

	###########################################################################

	function glob(g)
		r = read(`bash -c "ls -d -w1 $g | tee"`)
		s = chomp(String(r))
		if s == ""
			list = String[]
		else
			list = Vector{String}(split(s, "\n"))
		end
		return list
	end

	function check_file(file_name)
		if isdir(file_name)
			msg(FATAL, "Dir \"$file_name\" is not a file!")
		end
		if !isfile(file_name)
			msg(FATAL, "File \"$file_name\" does not exists!")
		end
	end

	@doc """
		Make name shorter by removing common part.
		@return common_name, nick_names
	"""
	function gen_nick_names(
		names::Vector{T},
		separator::AbstractString
	) where {
		T <: AbstractString
	}
		common_name = ""
		
		s1 = separator
		s2 = s1^2
		function recursive_split(name)
			splited_name = Vector{String}[]
			for n2 in split(name, s2)
				push!(splited_name, String[])
				for n1 in split(n2, s1)
					push!(splited_name[end], n1)
				end
			end
			return splited_name
		end
		function recursive_join(splited_name)
			j = [join(n2, s1) for n2 in splited_name]
			jf = filter(n1 -> n1 != "", j)
			name = join(jf, s2)
			return name
		end
		
		table = Dict{String, Int}()
		splited_names = map(recursive_split, names)
		for n in splited_names
			for n2 in n
				for n1 in n2
					if !haskey(table, n1)
						table[n1] = 1
					else
						table[n1] += 1
					end
				end
			end
		end
		
		N = length(names)
		common = Set{String}()
		for (n1, count) in table
			if count == N
				push!(common, n1)
			end
		end
		
		nick_names = String[]
		splited_common_name = Vector{String}[]
		splited_nick_names = Vector{Vector{String}}[]
		for n in splited_names
			push!(splited_nick_names, Vector{String}[])
			for (i2, n2) in enumerate(n)
				push!(splited_nick_names[end], String[])
				for n1 in n2
					if n1 in common
						# Add to common.
						while length(splited_common_name) < i2
							push!(splited_common_name, String[])
						end
						if !(n1 in splited_common_name[i2])
							push!(splited_common_name[i2], n1)
						end
					else
						# Add to nick name.
						push!(splited_nick_names[end][end], n1)
					end
				end
			end
		end
		
		
		common_name = recursive_join(splited_common_name)
		nick_names = map(recursive_join, splited_nick_names)
		
		return common_name, nick_names
	end
	
	###########################################################################

end
