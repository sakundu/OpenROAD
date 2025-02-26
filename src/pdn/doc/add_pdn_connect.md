# add_pdn_connect

## Synopsis
```
  % add_pdn_connect
    [-grid grid_name] \
    [-layers list_of_two_layers] \
    [-cut_pitch pitch_value] \
    [-fixed_vias list_of_fixed_vias] \
    [-max_rows integer] \
    [-max_columns integer] \
    [-ongrid list_of_layers] \
    [-split_cuts list_of_layers]
```

## Description

The `add_pdn_connect` command is used to define which layers in the power grid are to be connected together. During power grid generation, vias will be added for overlapping power nets and overlapping ground nets. The use of fixed vias from the technology file can be specified or else via stacks will be constructed using VIARULEs. If VIARULEs are not available in the technology, then fixed vias must be used.

The `-grid` argument defines the name of the grid to which this stripe specification will be added. If no `-grid` argument is specified, the connection will be added to the grid created with the previous [define_pdn_grid](define_pdn_grid.md) command.

The `-layers` argument defines which two layers are to be connected together. Where power stripes on these two layers overlap one or more vias are added in a vertical stack to connect the layers together - this is repeated for ground nets.If the area of the overlap does not cover the width of both layers, then a via stack will not be added.
Usually, the two layers are orthogonal to each other, but in the case of dual layer stdcell rails, the two layers overlap and a the `-cut_pitch` argument is used to specify the pitch of via placements along the conincident wires.
The `-fixed_vias` argument is used to specify a list of fixed vias defined in the technology file to build the vias stack between the specified layers.
The -max_rows and -max_columns options can be used to restrict the size of the via to be inserted at any given crossing, limiting the number of rows and columns in the array of cuts.
Connections made between non-adjacent layers in the metal stack will necessarily result in metal being added around the via cuts on each layer inbetween. The -ongrid option can be used to specify that metal added on the specified intermediate layer(s) should be on the routing grid.
The -split_cuts option specifies that single cut vias should be used on the crossing. Specify a list of layers, vias connecting to the specified layers will be split into single cut vias, rather than a single multi-cut via.

## Options

| Switch Name | Description |
| ----- | ----- |
| `-grid` | Specifies the name of the grid definition to which this connection will be added. (Default: Last grid created by `define_pdn_grid`) |
| `-layers` | Layers to be connected where there are overlapping power or overlapping ground nets |
| `-cut_pitch` | When the two layers are parallel e.g. overlapping stdcell rails, specify the distance between via cuts |
| `-fixed_vias` | list of fixed vias to be used to form the via stack |
| `-max_rows` | Specify a limit to the maximum number of rows in the via connection between the two layers |
| `-max_columns` | Specify a limit to the maximum number of columns in the via connection between the two layers |
| `-ongrid` | Force intermediate metal layers in a via stack to be on the routing grid |
| `-split_cuts` | A number of single cut vias will be used instead of a single multi-cut via |



## Examples
```
add_pdn_connect -grid main_grid -layers {metal1 metal2} -cut_pitch 0.16
add_pdn_connect -grid main_grid -layers {metal2 metal4}
add_pdn_connect -grid main_grid -layers {metal4 metal7}

add_pdn_connect -grid ram -layers {metal4 metal5}
add_pdn_connect -grid ram -layers {metal5 metal6}
add_pdn_connect -grid ram -layers {metal6 metal7}

add_pdn_connect -grid rotated_rams -layers {metal4 metal6}
add_pdn_connect -grid rotated_rams -layers {metal6 metal7}

```
