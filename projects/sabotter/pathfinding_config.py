set_max_path_size(20)
set_node_cost(50)

add_graph('pathfinding_graph', {
    'middle_top'  : (0, 1700),
    'middle_midh' : (0, 1200),
    'middle_midl' : (0, 800),
    'middle_bot'  : (0, 600),

    'orange_top'  : (650, 1700),
    'orange_midh' : (600, 1100),
    'orange_midl' : (550, 800),

    'orange_water_tower'          : (1200, 1650),
    'orange_water_dispenser_near' : (1200, 1200),
    'orange_bot_corner'           : (1000, 600),
    'orange_bee'                  : (1200, 300),

    'orange_water_dispenser_far'  : (-890, 400),

    'green_top'   : (-650, 1700),
    'green_midh'  : (-600, 1100),
    'green_midl'  : (-550, 800),

    'green_water_tower'          : (-1200, 1650),
    'green_water_dispenser_near' : (-1200, 1200),
    'green_bot_corner'           : (-1000, 600),
    'green_bee'                  : (-1200, 300),

    'green_water_dispenser_far'  : (890, 400),

    'a' : (1000,1500),
    'b' : (300,1500),
    'c' : (-300,1500),
    'd' : (-1000,1500),
    'e' : (900,900),
    'f' : (300,1000),
    'g' : (-300,1000),
    'h' : (-900,900),

  }, [
  #vertical vertexes
  'middle_top middle_midh middle_midl middle_bot',
  'orange_midh orange_midl',
  'green_midh green_midl',
  'orange_water_tower orange_water_dispenser_near',
  'orange_bot_corner orange_bee',
  'green_water_tower green_water_dispenser_near',
  'green_bot_corner green_bee',

  #horizontal vertexes
  'green_water_tower          green_top  middle_top  orange_top  orange_water_tower',
  'green_water_dispenser_near green_midh middle_midh orange_midh orange_water_dispenser_near',
  'green_bot_corner           green_midl middle_midl orange_midl orange_bot_corner',
  'green_bee orange_water_dispenser_far',
  'orange_bee green_water_dispenser_far',

  #diagonals
  'green_water_tower           d green_midh g middle_midl',
  'green_water_dispenser_near  h green_midl middle_bot',
  'green_bee  orange_water_dispenser_far green_midl g middle_midh b orange_top',
  'green_bot_corner            h green_midh c middle_top',
  'green_water_dispenser_near  d green_top',
  'green_bot_corner orange_water_dispenser_far',
  'orange_bot_corner green_water_dispenser_far',

  'orange_water_tower          a orange_midh f middle_midl',
  'orange_water_dispenser_near e orange_midl middle_bot',
  'orange_bee  green_water_dispenser_far orange_midl f middle_midh c green_top',
  'orange_bot_corner           e orange_midh b middle_top',
  'orange_water_dispenser_near a orange_top',

  #shortcuts
  #'green_water_dispenser_near green_water_dispenser_far',
  #'green_top orange_midh',
  #'green_top orange_midl',
  #'orange_water_dispenser_near orange_water_dispenser_far',

  ])

