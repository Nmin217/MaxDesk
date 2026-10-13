// Runs scenario scripts against the REAL simulator code (headless, no browser).
//   node tests/sim/run.js cord                 one scenario from tests/sim/scenarios/
//   node tests/sim/run.js slots parking        several, one after another in one process
// Scenarios print their own results. Exit code 1 if the sim throws.
const fs = require('fs'), path = require('path');
require('./env.js');
const html = fs.readFileSync(path.join(__dirname, '../../robot/simulator/summon_table_sim.html'), 'utf8');
const sim = html.match(/<script>([\s\S]*?)<\/script>/)[1];
const simCore = sim.slice(0, sim.indexOf('// ============ 3D concept model'));
const common = fs.readFileSync(path.join(__dirname, 'common.js'), 'utf8');
const names = process.argv.slice(2);
if (!names.length) { console.error('usage: node tests/sim/run.js <scenario> [...]'); process.exit(2); }
const scen = names.map(n => '{\n' + fs.readFileSync(path.join(__dirname, 'scenarios', n + '.js'), 'utf8') + '\n}').join('\n');   // one block each, so names do not collide
eval(simCore + '\n' + common + '\n' + scen);
