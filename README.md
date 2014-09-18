Dijkstra
========

Algorithm::DijkstraSP aka Dijkstra's Shortest Path Alogrithm - Developed using Perl

=head1 SYNOPSIS

    # Instance creation
    my $obj = Algorithm::DijkstraSP->new( map_file => '/tmp/inputmap.txt');
    my $obj = Algorithm::DijkstraSP->new( map_arrayref => [
                                                        ['Munchen', 'Ingolstadt', '80'],
                                                        ['Augsburg,', 'Ulm', '8'],
                                                        ['Ingolstadt,', 'Straubing,', '124'],
                                                        .
                                                        .
                                                        .
                                                      ]);

    # Interface to load map data after instance creation
    $obj->loadMapFile("/path/filename.txt");
    $obj->loadMapArrayRef([[] [] []]);

    # To Set the root Vertex of the Dijkstra's algorithm
    $obj->rootVertex('Munchen');

    # Beneficial functions using Dijkstra's algorithm
    print $obj->allVertices;                   # returns ('Munchen','Ingolstadt', 'Augsburg', 'Ulm', 'Straubing')

    print $obj->shortestPath('Straubing');     # returns ('Munchen', 'Augsburg', 'Ulm', 'Straubing') # Array
    print $obj->shortestDistance('Straubing'); # returns 206 # Scalar
    $obj->listShortestPaths();                 # returns the shortest path to all other vertices
    $obj->listShortestDistances();             # returns the shortest distance to all other vertices

    print $obj->displayShortestRoutes();       # Displays the shortest path, distance in verbose

