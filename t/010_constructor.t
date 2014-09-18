use Test::More (tests => 6);
use Test::Exception;

use_ok('Algorithm::DijkstraSP');

$map_array =  [
  ['Chennai', 'Pondy', 120],
  ['Madurai', 'Chennai', 220],
  ['Madurai', 'Theni', 45],
  ['Chennai', 'Dindigul', 400],
  ['Dindigul', 'Theni', 60],
  ['Pondy', 'Dindigul', 110],
];

new_ok('Algorithm::DijkstraSP' => []);
new_ok('Algorithm::DijkstraSP' => [map_file => '../data/input.txt']);
new_ok('Algorithm::DijkstraSP' => [map_arrayref => $map_array]);

$map_array_fail =  [
  ['Chennai', 'Pondy', 120],
  ['Madurai', 'Chennai', 220],
  ['Madurai', 'Theni', 45],
  ['Chennai', 'Dindigul', 400],
  ['Dindigul', 'Theni'],
  ['Pondy', 'Dindigul', 110],
];
dies_ok { Algorithm::DijkstraSP->new('map_file' => '/some/invalid/file.txt') } 'map_file: is a invalid map file';
dies_ok { Algorithm::DijkstraSP->new('map_arrayref' => $map_array_fail) } 'map_arrayref: is a invalid map array';

