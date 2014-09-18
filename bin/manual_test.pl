#!/usr/bin/perl

BEGIN {
    push (@INC,"../lib/");
}  
   

use strict;
use Graph::Undirected;
use Data::Dumper;
use Algorithm::DijkstraSP;


my ($FileName, $City, $DestCity) = @ARGV;

$FileName = "../data/input.txt";
$City     = 'Munchen';
$DestCity = 'Heidelberg';

my $adj = Algorithm::DijkstraSP->new('map_file' => $FileName);
#$adj->loadMapFile($FileName);

#print Dumper [ $adj->allVertices() ];
$adj->rootVertex($City);

print "Shortest route path from city $City\n";
print Dumper [$adj->listShortestPaths()];
print "================================================================\n";

print "Shortest route path distance from city $City\n";
print Dumper [$adj->listShortestDistances()];
print "================================================================\n";

print "Shortest route path and distance from $City to $DestCity\n";
print Dumper [$adj->shortestPath($DestCity)];
print Dumper [$adj->shortestDistance($DestCity)];
print "================================================================\n";

print "Shortest route path and distance from $City to Ulm\n";
print Dumper [$adj->shortestPath('Ulm')];
print Dumper [$adj->shortestDistance('Ulm')];
print "================================================================\n";

print "Display all Shortest Routes from city $City\n";
$adj->displayShortestRoutes();
print "================================================================\n";

