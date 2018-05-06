set_max_path_size(10)
set_node_cost(50)

add_graph('pathfinding_graph', {
    'middle_top'  : (0, 1650),
    'middle_midh' : (0, 1200),
    'middle_midl' : (0, 800),
    'middle_bot'  : (0, 550),

    'orange_top'  : (600, 1650),
    'orange_midh' : (600, 1200),
    'orange_midl' : (600, 800),
    'orange_bot'  : (600, 550),

    'orange_water_tower'          : (1200, 1650),
    'orange_water_dispenser_near' : (1200, 1200),
    'orange_bot_corner'           : (1200, 700),
    'orange_bee'                  : (1200, 300),

    'orange_water_dispenser_far'  : (-890, 400),
    'orange_water_recycle'        : (-300, 450),

    'green_top'   : (-600, 1650),
    'green_midh'  : (-600, 1200),
    'green_midl'  : (-600, 800),
    'green_bot'   : (-600, 550),

    'green_water_tower'          : (-1200, 1650),
    'green_water_dispenser_near' : (-1200, 1200),
    'green_bot_corner'           : (-1200, 700),
    'green_bee'                  : (-1200, 300),

    'green_water_dispenser_far'  : (890, 400),
    'green_water_recycle'        : (300, 450),

  }, [
  #vertical vertexes
  'middle_top middle_midh middle_midl middle_bot',
  'orange_top orange_midh orange_midl orange_bot',
  'green_top green_midh green_midl green_bot',
  'orange_water_tower orange_water_dispenser_near orange_bot_corner orange_bee',
  'green_water_tower green_water_dispenser_near green_bot_corner green_bee',

  #horizontal vertexes
  'green_water_tower          green_top  middle_top  orange_top  orange_water_tower',
  'green_water_dispenser_near green_midh middle_midh orange_midh orange_water_dispenser_near',
  'green_bot_corner           green_midl middle_midl orange_midl orange_bot_corner',
  'green_bot  middle_bot  orange_bot',
  'green_bee orange_water_dispenser_far orange_water_recycle middle_bot',
  'orange_bee green_water_dispenser_far green_water_recycle middle_bot',
  'orange_water_recycle green_water_recycle',

  #diagonals
  'green_water_tower           green_midh middle_midl orange_bot',
  'green_water_dispenser_near  green_midl middle_bot',
  'green_bot_corner            green_bot',
  'green_bee  orange_water_dispenser_far green_midl middle_midh orange_top',
  'green_bot_corner            green_midh middle_top',
  'green_water_dispenser_near  green_top',
  'green_water_recycle orange_bot',
  'green_bot_corner orange_water_dispenser_far green_bot',
  'orange_bot_corner green_water_dispenser_far orange_bot',

  'orange_water_tower          orange_midh middle_midl green_bot',
  'orange_water_dispenser_near orange_midl middle_bot',
  'orange_bot_corner           orange_bot',
  'orange_bee  green_water_dispenser_far orange_midl middle_midh green_top',
  'orange_bot_corner           orange_midh middle_top',
  'orange_water_dispenser_near orange_top',
  'orange_water_recycle green_bot',

  #shortcuts
  #'green_water_dispenser_near green_water_dispenser_far',
  #'green_top orange_midh',
  #'green_top orange_midl',
  #'orange_water_dispenser_near orange_water_dispenser_far',

  ])

