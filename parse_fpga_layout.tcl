# source inside of vivado

# parse arguments
set FLORA_DIR "$::env(CP_DIR)/implementation/floorplanner"
if { ![info exists out_name] } {
  if { ![info exists ::env(out_name)] } {
    set out_name "pynq"
  } else {
    set out_name $::env(out_name)
  }
}
if { ![info exists row] } {
  if { ![info exists ::env(row)] } {
    set row 155
  } else {
    set row $::env(row)
  }
}

set out_name_upper [string toupper "${out_name}"]

# open file and write preamble
set out_file [open "${FLORA_DIR}/include/${out_name}_fine_grained.h" "w"]
puts $out_file "\n#ifndef __${out_name_upper}_FINE_GRAINED_H__"
puts $out_file "#define __${out_name_upper}_FINE_GRAINED_H__\n"
puts $out_file "#include \"fine_grained.h\""
puts $out_file "#include \"${out_name}.h\"\n"
puts $out_file "class ${out_name}_fine_grained \{"
puts $out_file "public:"
puts $out_file "    finegrained_res_description fg\[${out_name_upper}_WIDTH+1\] = \{"

# get tiles (sorted by column)
set filter "ROW==${row}"
set tiles [get_tiles -filter $filter]

foreach t $tiles {
  set do_write_tile false
  set tile_type "CLB"
  set slice_x0 0
  set slice_x1 0
  set tile_class "CLASS_NONE"
  set tile_alignment "ALIGNMENT_L"

  # get tile type and class
  switch -regexp $t {
    {CLBL[LM]_[LR]_X[0-9]*Y[0-9]*} {
      set tile_type "CLB"
      set tile_class [string range $t 0 4]
      set do_write_tile true
    }
    {CLE[LM]_[LR]_X[0-9]*Y[0-9]*} {
      set tile_type "CLB"
      set tile_class "CLBL[string range $t 3 3]"
      set do_write_tile true
    }
    {BRAM_[LR]_X[0-9]*Y[0-9]*} {
      set tile_type "BRAM"
      set do_write_tile true
    }
    {BRAM_X[0-9]*Y[0-9]*} {
      set tile_type "BRAM"
      set do_write_tile true
    }
    {DSP_[LR]_X[0-9]*Y[0-9]*} {
      set tile_type "DSP"
      set do_write_tile true
    }
    {DSP_X[0-9]*Y[0-9]*} {
      set tile_type "DSP"
      set do_write_tile true
    }
  }

  if { $do_write_tile } {
    # get tile alignment
    switch -regexp $t {
      {.*_R_X[0-9]*Y[0-9]*} {
        set tile_alignment "ALIGNMENT_R"
      }
    }

    # get slice indices inside tile
    set slices [get_sites -of $t -filter {NAME !~ TIEOFF*}]
    set slice_x0 100000000
    set slice_x1 0
    foreach s $slices {
      set slice_x [lindex [split [lindex $s 0] XY] end-1]
      if { $slice_x < $slice_x0 } { set slice_x0 $slice_x }
      if { $slice_x > $slice_x1 } { set slice_x1 $slice_x }
    }
    puts "Slices $slices => $slice_x0, $slice_x1"

    puts $out_file "        \{${tile_type}, ${slice_x0}, ${slice_x1}, ${tile_class}, ${tile_alignment}\},"
  }
}

# write postamble and close file
puts $out_file "        \{FBDN, 0, 0, CLASS_NONE, ALIGNMENT_L\}"
puts $out_file "    };\n"
puts $out_file "    void init_fine_grained();"
puts $out_file "    ${out_name}_fine_grained();\n"
puts $out_file "};\n"
puts $out_file "#endif // __${out_name_upper}_FINE_GRAINED_H__\n"
close $out_file
