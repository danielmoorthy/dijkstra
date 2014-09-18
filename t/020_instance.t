use Test::More (tests => 6);

use_ok('Algorithm::DijkstraSP');

subtest 'setting valid rootVertex ' => sub {
    my $oc = new_ok('Algorithm::DijkstraSP' => [map_file => '../data/input.txt']);
    my $ro = 'Munchen';
    $oc->rootVertex($ro);
    is($oc->rootVertex(), $ro, 'Setting root vertex successful');
};

subtest 'list shortest paths' => sub {
    my $oc = new_ok('Algorithm::DijkstraSP' => [map_file => '../data/input.txt']);
    my $ro = 'Munchen';
    $oc->rootVertex($ro);
    is (ref([$oc->listShortestPaths()]), 'ARRAY', 'list shortest paths');
};

subtest 'list shortest distances' => sub {
    my $oc = new_ok('Algorithm::DijkstraSP' => [map_file => '../data/input.txt']);
    my $ro = 'Munchen';
    $oc->rootVertex($ro);
    is (ref([$oc->listShortestDistances()]), 'ARRAY', 'list shortest distances');
};

subtest 'get shortest path' => sub {
    my $oc = new_ok('Algorithm::DijkstraSP' => [map_file => '../data/input.txt']);
    my $ro = 'Munchen';
    $oc->rootVertex($ro);
    is (ref([$oc->shortestPath('Ulm')]), 'ARRAY', 'list shortest path');
};

subtest 'list shortest distance' => sub {
    my $oc = new_ok('Algorithm::DijkstraSP' => [map_file => '../data/input.txt']);
    my $ro = 'Munchen';
    $oc->rootVertex($ro);
    is ($oc->shortestDistance('Ulm'), '76', 'list shortest distance');
};
