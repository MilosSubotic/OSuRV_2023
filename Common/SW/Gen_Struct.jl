
###############################################################################

module Gen_Struct
	export
		@must_specify,
		@gen_struct_with_must_specify,
		@gen_struct_with_default_nothing

	###########################################################################

	import Utils: @must_specify
	
	###########################################################################

	# Use something like this to find out how to make Expr() tree.
	#	ex = Meta.parse("""
	#		struct Spec
	#			full_bridge::Bool
	#		end
	#	""")

	function indent_new_line_print(x)
		b = IOBuffer()
		print(b, x)
		s = String(take!(b))
		s2 = replace(s, "\n" => "\n\t")
		return s2
	end

	function gen_show(name, field_type_arr...)
		#println(io, "\t", "val", " = ", s.val)
		fields_exprs = Expr[]
		for i in 1:length(field_type_arr)>>1
			field = field_type_arr[(i-1)*2+1]
			push!(
				fields_exprs,
				Expr(
					:call,
					:println,
					:io,
					"\t$field = ",
					Expr(
						:call,
						Expr(
							:.,
							:Gen_Struct,
							:(:indent_new_line_print)
						),
						Expr(
							:.,
							:s,
							QuoteNode(field)
						)
					),
					","
				)
			)
		end
		
		ex = Expr(
			:function,
			Expr(
				:call,
				Expr(
					:.,
					:Base,
					:(:show)
				),
				Expr(
					:(::),
					:io,
					:IO
				),
				Expr(
					:(::),
					:s,
					name
				)
			),
			Expr(
				:block,
				Meta.parse("""println(io, "$name(")"""),
				fields_exprs...,
				Meta.parse("print(io, ')')"),
				Meta.parse("return io"),
			)
		)
		return ex
	end
	
	function gen_map(name, field_type_arr...)
		fields_exprs = Expr[]
		for i in 1:length(field_type_arr)>>1
			field = field_type_arr[(i-1)*2+1]
			push!(
				fields_exprs,
				Expr(
					:call,
					:f,
					Expr(
						:.,
						:s,
						QuoteNode(field)
					)
				)
			)
		end
		
		ex = Expr(
			:function,
			Expr(
				:call,
				Expr(
					:.,
					:Base,
					:(:map)
				),
				:f,
				Expr(
					:(::),
					:s,
					name
				)
			),
			Expr(
				:block,
				Expr(
					:return,
					Expr(
						:vect,
						fields_exprs...,
					)
				)
			)
		)
		return ex
	end

	function _gen_struct_with_must_specify(name, field_type_arr...)
		@assert length(field_type_arr) % 2 == 0
		
		ft = []
		for i in 1:length(field_type_arr)>>1
			push!(
				ft,
				(
					field_type_arr[(i-1)*2+1],
					field_type_arr[(i-1)*2+2]
				)
			)
		end
		
		b = []
		for (f, t) in ft
			push!(b, Expr(:(::), f, t))
		end
		struct_ex = Expr(
			:struct,
			true,
			name,
			Expr(:block, b...)
		)
		
		
		kws = []
		args = []
		for (f, t) in ft
			push!(
				kws,
				Expr(
					:kw,
					Expr(
						:(::),
						f,
						t
					),
					Expr(
						:macrocall,
						Symbol("@must_specify"),
						f
					)
				)
			)
			push!(
				args,
				f
			)
		end
		ctor_ex = Expr(
			:function,
			Expr(
				:call,
				name,
				Expr(
					:parameters,
					kws...
				)
			),
			Expr(
				:block,
				Expr(
					:call,
					name,
					args...
				)
			)
		)
		
		ex = Expr(
			:block,
			struct_ex,
			ctor_ex,
			gen_show(name, field_type_arr...),
			gen_map(name, field_type_arr...),
		)
		
		return ex
	end
	
	macro gen_struct_with_must_specify(name, field_type_arr...)
		quote
			$__module__.eval(
				_gen_struct_with_must_specify(
					$name, $(field_type_arr...)
				)
			)
		end
	end
	
	function _gen_struct_with_default_nothing(name, field_type_arr...)
		@assert length(field_type_arr) % 2 == 0
		
		ft = []
		for i in 1:length(field_type_arr)>>1
			push!(ft,
				(
					field_type_arr[(i-1)*2+1],
					field_type_arr[(i-1)*2+2]
				)
			)
		end
		
		b = []
		for (f, t) in ft
			push!(
				b,
				Expr(
					:(::),
					f,
					Expr(
						:curly,
						:Union,
						t,
						:Nothing
					),
				)
			)
		end
		struct_ex = Expr(
			:struct,
			true,
			name,
			Expr(:block, b...)
		)
		
		
		kws = []
		args = []
		for (f, t) in ft
			push!(
				kws,
				Expr(
					:kw,
					Expr(
						:(::),
						f,
						Expr(
							:curly,
							:Union,
							t,
							:Nothing
						),
					),
					:nothing
				)
			)
			push!(
				args,
				f
			)
		end
		ctor_ex = Expr(
			:function,
			Expr(
				:call,
				name,
				Expr(
					:parameters,
					kws...
				)
			),
			Expr(
				:block,
				Expr(
					:call,
					name,
					args...
				)
			)
		)
		
		ex = Expr(
			:block,
			struct_ex,
			ctor_ex,
			gen_show(name, field_type_arr...),
			gen_map(name, field_type_arr...),
		)
		
		return ex
	end
	
	macro gen_struct_with_default_nothing(name, field_type_arr...)
		quote
			$__module__.eval(
				_gen_struct_with_default_nothing(
					$name, $(field_type_arr...)
				)
			)
		end
	end
	
end

###############################################################################
