package Algorithm::DijkstraSP;

BEGIN {
  $Algorithm::DijkstraSP::VERSION = '0.01';
}

use strict;
use warnings;
use Carp;
use Graph::Undirected;


=head1 NAME

Algorithm::DijkstraSP aka Dijkstra's Shortest Path Alogrithm

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

=cut

=head1 DESCRIPTION

Illustration of Dijkstra's algorithm search for finding path from a start node (lower left, red) to a goal node (upper right, green) in a robot motion planning problem. Open nodes represent the "tentative" set. Filled nodes are visited ones, with color representing the distance: the greener, the farther. Nodes in all the different directions are explored uniformly, appearing as a more-or-less circular wavefront as Dijkstra's algorithm uses a heuristic identically equal to 0.

Let the node at which we are starting be called the initial node. Let the distance of node Y be the distance from the initial node to Y. Dijkstra's algorithm will assign some initial distance values and will try to improve them step by step.

1. Assign to every node a tentative distance value: set it to zero for our initial node and to infinity for all other nodes.

2. Mark all nodes unvisited. Set the initial node as current. Create a set of the unvisited nodes called the unvisited set consisting of all the nodes.

3. For the current node, consider all of its unvisited neighbors and calculate their tentative distances. For example, if the current node A is marked with a distance of 6, and the edge connecting it with a neighbor B has length 2, then the distance to B (through A) will be 6 + 2 = 8. If this distance is less than the previously recorded tentative distance of B, then overwrite that distance. Even though a neighbor has been examined, it is not marked as "visited" at this time, and it remains in the unvisited set.

4. When we are done considering all of the neighbors of the current node, mark the current node as visited and remove it from the unvisited set. A visited node will never be checked again.

5. If the destination node has been marked visited (when planning a route between two specific nodes) or if the smallest tentative distance among the nodes in the unvisited set is infinity (when planning a complete traversal; occurs when there is no connection between the initial node and remaining unvisited nodes), then stop. The algorithm has finished.

6. Select the unvisited node that is marked with the smallest tentative distance, and set it as the new "current node" then go back to step 3.

 ....... Some more description about this module ......

=cut

# Some Class Constants
$Algorithm::DijkstraSP::INFINITE_VALUE = 9999999999;
$Algorithm::DijkstraSP::INITIAL_WEIGHT = 0;

# Class Constructor
sub new
{
   my ($proto, %inhash) = @_;
   my ($self,  $class);

   $class = ref($proto) || $proto;
   $self  = {};
   bless($self, $class);

   $self->_initialise(%inhash);
   return $self;
}

sub _initialise
{ 
    my ($self, %inHash) = @_;

    $self->_mapDataAvailable(0);
    if (exists ($inHash{'map_file'}) and exists($inHash{'map_arrayref'})) {
	croak "ERROR: Specify either map_file or map_arrayref. Not both!!";
    }

    if (defined ($inHash{'map_file'})) {
        if ( !-e $inHash{'map_file'}) {
	   croak "ERROR: Invalid map file name supplied. File $inHash{'map_file'} doesnot exists!!";
        }
        $self->{'map_file'} = $inHash{'map_file'};
        $self->_parseMapFile();
    } elsif (defined ($inHash{'map_arrayref'})) {
        if ( ref($inHash{'map_arrayref'} ne 'ARRAY')) {
	   croak "ERROR: Invalid map array reference supplied. Value to 'map_arrayref' must be an array reference!!";
        }
        $self->{'map_arrayref'} = $inHash{'map_arrayref'};
        $self->_parseMapArrayRef();
    }

    if ($self->_mapDataAvailable()) {
       $self->_loadGraph();
    }
}

sub _loadGraph
{
    my $self = shift;

    my $gObj = Graph::Undirected->new(compat02 => 1);
    my ($rec, $lNode, $rNode, $weight);
    foreach $rec (@{$self->_mapData()}) { 
      ($lNode, $rNode, $weight) = @{$rec};
      $gObj->add_weighted_path($lNode, $weight, $rNode);
    }

    $self->_graphObj($gObj);
}

sub _graphObj
{
    my ($self, $gObj) = @_;

    if (defined $gObj) {
       $self->{'graph_object'} = $gObj;
    } else {
       return $self->{'graph_object'};
    }
}

sub loadMapFile
{
   my ($self, $mFile) = @_;

   return $self->_initialise('map_file' => $mFile);
}

sub loadMapArrayRef
{
    my ($self, $mData) = @_;

    return $self->_initialise('map_arrayref' => $mData);
}

sub rootVertex
{
    my ($self, $rVertex) = @_;

    if (defined ($rVertex)) {
       if (!$self->isValidVertex($rVertex)) {
          croak "ERROR: Invalid Root Vertex [$rVertex]!!";
       }
       $self->{'root_vertex'} = $rVertex;
       $self->_executeDijkstra();
    } else {
       return $self->{'root_vertex'};
    }
}

sub isValidVertex
{
    my ($self, $mVertex) = @_;

    return 0 if (!defined $mVertex);
    return $self->_graphObj->has_vertex($mVertex);
}

sub allVertices
{
    my $self = shift;
    return $self->_graphObj->vertices();
}

sub _initAlgorithm
{
    my ($self, $beginVertex) = @_;

    # All vertices in a Graph
    my %allVertices;
    map { $allVertices{$_} = $Algorithm::DijkstraSP::INFINITE_VALUE; } $self->allVertices();
    $self->{'_instance_'}->{'vxAll'} = {%allVertices};

    # Initialise Algo Variables
    $self->{'_instance_'}->{'vxSolved'}     = {};
    $self->{'_instance_'}->{'vxUnSolved'}   = {};
    $self->{'_instance_'}->{'vxPrev'}       = {};
    $self->{'_instance_'}->{'vxDistance'}   = {%allVertices};

    # Initialise Interation Variables
    $self->_setVxDistance($beginVertex, $Algorithm::DijkstraSP::INITIAL_WEIGHT);
    $self->_addVxUnSolved($beginVertex);
}

sub _getVxPrev
{
    my ($self, $pVertex) = @_;

    return $self->{'_instance_'}->{'vxPrev'}->{$pVertex};
}

sub _setVxPrev
{
    my ($self, $pVertex, $sVertex) = @_;

    $self->{'_instance_'}->{'vxPrev'}->{$pVertex} = $sVertex;
}

sub _getVxDistance
{
    my ($self, $gVertex) = @_;

    return $self->{'_instance_'}->{'vxDistance'}->{$gVertex};
}

sub _setVxDistance
{
    my ($self, $gVertex, $distValue) = @_;

    $self->{'_instance_'}->{'vxDistance'}->{$gVertex} = $distValue;
}

sub _listVxUnSolved
{
    my $self = shift;

    return $self->{'_instance_'}->{'vxUnSolved'};
}

sub _addVxUnSolved
{
   my $self = shift;

   $self->{'_instance_'}->{'vxUnSolved'}->{shift()} = undef;
}

sub _deleteVxUnSolved
{
   my $self = shift;

   my $dVertex = shift;
   if (exists ($self->{'_instance_'}->{'vxUnSolved'}->{$dVertex})) {
      delete ($self->{'_instance_'}->{'vxUnSolved'}->{$dVertex});
   }
}

sub _listVxSolved
{
    my $self = shift;

    return $self->{'_instance_'}->{'vxSolved'};
}

sub _addVxSolved
{
   my $self = shift;

   $self->{'_instance_'}->{'vxSolved'}->{shift()} = undef;
}

sub _isVxSolved
{
   my $self = shift;

   my $dVertex = shift;
   return exists ($self->{'_instance_'}->{'vxSolved'}->{$dVertex});
}

sub _executeDijkstra
{
    my ($self, $sVertex) = @_;

    if (!defined ($sVertex)) {
        $sVertex = $self->rootVertex();
    }
    # Initialise Algorithm
    $self->_initAlgorithm($sVertex);

    my ($iVertex);

    while (scalar(keys(%{$self->_listVxUnSolved()}))) {
       $iVertex = $self->_getMinimumVertex();
       $self->_addVxSolved($iVertex);
       $self->_deleteVxUnSolved($iVertex);
       $self->_calculateMinimumDistance($iVertex);
    }

    return;
}

sub _calculateMinimumDistance
{
    my ($self, $cVertex) = @_;

    my ($iVertex,  $estDistance);
    foreach $iVertex ($self->_getUnSolvedAdjacentVertices($cVertex)) {

       $estDistance = $self->_getVxDistance($cVertex) +
                      $self->_graphObj->get_attribute('weight', $cVertex, $iVertex);
       if ($self->_getVxDistance($iVertex) > $estDistance) {
          $self->_setVxDistance($iVertex, $estDistance);
          $self->_addVxUnSolved($iVertex);
          $self->_setVxPrev($iVertex, $cVertex);
       }
    }
}

sub _getUnSolvedAdjacentVertices
{
    my ($self, $sVertex) = @_;

    my (@aVertices, $iVertex) = ();
    foreach $iVertex ($self->allVertices()) {
       if ($self->_graphObj->has_edge($sVertex, $iVertex) and !$self->_isVxSolved($iVertex)) {
          push (@aVertices, $iVertex);
       }
    }

    return @aVertices;
}

sub _getMinimumVertex
{
    my $self = shift;
    my ($minVertex, $iVertex) = (undef);

    foreach $iVertex (keys(%{$self->_listVxUnSolved()})) {
       if (!defined($minVertex)) {
          $minVertex = $iVertex;
       } else {
         if ($self->_getVxDistance($iVertex) < $self->_getVxDistance($minVertex)) {
            $minVertex = $iVertex
         }
       }
    }

    return $minVertex;
}

sub _getLinkedPath
{
    my ($self, $dVertex) = @_;

    my $sVertex = $self->rootVertex();
    my $lVertex = $dVertex;
    my (@pathArray) = ();
 
    push (@pathArray, $lVertex);
    while (defined ($self->_getVxPrev($lVertex))) {
       $lVertex = $self->_getVxPrev($lVertex);
       push (@pathArray, $lVertex);
    }

    return reverse @pathArray;
}

sub shortestPath
{
    my ($self, $dVertex) = @_;

    if (!$self->isValidVertex($dVertex)) {
       croak "ERROR: Invalid End Vertex [$dVertex]!!";
    }

    return $self->_getLinkedPath($dVertex);
}

sub shortestDistance
{
    my ($self, $dVertex) = @_;

    if (!$self->isValidVertex($dVertex)) {
       croak "ERROR: Invalid End Vertex [$dVertex]!!";
    }

    return $self->_getVxDistance($dVertex);
}

sub displayShortestRoutes
{
    my ($self) = @_;

    my @allVertices = $self->allVertices();
    my @path        = ();
    my $rootVertex  = $self->rootVertex();

    foreach my $iVertex (@allVertices) {

       print "From $rootVertex to $iVertex: ";
       @path = $self->_getLinkedPath($iVertex);

       next if (scalar(@path) <= 1);
       my ($sV, $eV, $di);
       print $path[0], " ";
       for (my $i=0; $i<@path-1; $i++) {
           $sV = $path[$i];
           $eV = $path[$i+1];
           $di = $self->_graphObj->get_attribute('weight', $sV, $eV);
           print "-($di)- $eV ";
       }
       print " = [Distance is ", $self->_getVxDistance($iVertex), "]\n";
    }

}

sub listShortestPaths
{
    my ($self) = @_;

    my @allVertices = $self->allVertices();
    my @allPaths    = ();
    foreach my $iVertex (@allVertices) {
       push (@allPaths, [$self->_getLinkedPath($iVertex)]);
    }

    return @allPaths;
}

sub listShortestDistances
{
    my ($self) = @_;

    my @allVertices = $self->allVertices();
    my (@allPaths, $dVertex, $cDistance) = ();
    foreach my $iVertex (@allVertices) {
       my @path = $self->_getLinkedPath($iVertex);

       $dVertex   = $path[-1];
       $cDistance = $self->_getVxDistance($dVertex);
       push(@allPaths, [$self->rootVertex(), $dVertex, $cDistance]);
    }

    return @allPaths;
}

sub _mapDataAvailable
{
    my ($self, $avlblFlag) = @_;

    if (defined $avlblFlag) {
       $self->{'map_data_avlbl'} = $avlblFlag;
    } else {
       return $self->{'map_data_avlbl'};
    }
}

sub _mapData
{
    my ($self, $mapData) = @_;

    if (defined $mapData) {
       $self->{'map_data'} = $mapData;
       $self->_mapDataAvailable(1);
    } else {
       return $self->{'map_data'};
    }
}

sub _parseMapArrayRef
{
    my ($self) = @_;

    my $mapArrayRef = $self->{'map_arrayref'};
    my $rec;
    foreach $rec (@{$mapArrayRef}) {
        if (@{$rec} != 3) {
	   croak "ERROR: Invalid map array reference supplied. Row with only 3 columns should be supplied!!";
        }
    }

    $self->_mapData($mapArrayRef);
    return;
}

sub _parseMapFile
{
    my $self = shift;
  
    my $mapFile = $self->{'map_file'};
  
    open (fH, "< $mapFile") or die "Can't open $mapFile : $!";
    my @fileData = <fH>;
    close(fH);
  
    my ($lNode, $rNode, $weight, $rec);
    my $mapData;
    foreach $rec (@fileData) {
       chomp($rec);
       ($lNode, $rNode, $weight) = split(/,\s+/, $rec);
       next if ($lNode eq '');
       push (@{$mapData}, [$lNode, $rNode, $weight]);
    }
  
    $self->_mapData($mapData);
    return;
}

=head1 ACKNOWLEDGEMENT

This module is only functional. May still require lot of finetuning related to accessing various datastructres used internally, code cleanup with respect to syntax etc.

This module is not benchmarked against performance testing standards.

Treat this module only for educational purpose and I admit that at its current state not to be fit for production purposes.
=cut

=head1 AUTHOR

Daniel Moorthy (r.moorthy@gmail.com)
=cut

=head1 COPYRIGHT AND LICENSE

Some Copyright information here.
=cut

=head1 LICENSE

This program is free software; you can redistribute it and/or modify it under the same terms as Perl 5 itself.

=cut

1;
