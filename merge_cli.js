/* =====================================================================
   ΕΝΟΠΟΙΗΣΗ ΑΠΟ ΓΡΑΜΜΗ ΕΝΤΟΛΩΝ — merge_cli.js

   Το merge_dxf.js μέχρι τώρα καλούνταν ΜΟΝΟ από τον browser (index.html).
   Αυτό το wrapper το κάνει διαθέσιμο και στη μαζική εκτέλεση, ώστε το
   TIDY_ALL.bat να βγάζει ΕΝΑ ενοποιημένο DXF και όχι μόνο χωριστά αρχεία.

   ΧΡΗΣΗ
       node merge_cli.js ΕΞΟΔΟΣ.dxf  αρχειο1.dxf αρχειο2.dxf ...

   Η σειρά/στάθμη κάθε αρχείου βγαίνει από το ΟΝΟΜΑ του (or0, or1, ...),
   όπως ακριβώς και στην έκδοση του browser. Τα αρχεία λεπτομερειών
   («Λεπτ_Υποστ») αναγνωρίζονται επίσης από το όνομα.
   ===================================================================== */
'use strict';

const fs = require('fs');
const path = require('path');

const M = require(path.join(__dirname, 'merge_dxf.js'));

const args = process.argv.slice(2);
if (args.length < 2) {
  console.error('Χρήση: node merge_cli.js ΕΞΟΔΟΣ.dxf αρχειο1.dxf [αρχειο2.dxf ...]');
  process.exit(1);
}

const outPath = args[0];
const inPaths = args.slice(1);

const items = [];
for (const p of inPaths) {
  if (!fs.existsSync(p)) {
    console.error('ΠΡΟΣΟΧΗ: δεν βρέθηκε ' + p);
    continue;
  }
  // Η στάθμη βγαίνει από το ΟΝΟΜΑ (…_or0, …_or1). Τα τακτοποιημένα αρχεία
  // λέγονται «<όνομα>_tidy.dxf», οπότε το επίθεμα πρέπει να αφαιρεθεί -
  // αλλιώς το «_tidy» κρύβει το «_or1» και η ενοποίηση αποτυγχάνει με
  // «Κανένα αρχείο με έγκυρο όνομα στάθμης».
  const cleanName = path.basename(p).replace(/_tidy(?=\.dxf$)/i, '')
                                    .replace(/_tidied(?=\.dxf$)/i, '');
  items.push({
    name: cleanName,
    text: fs.readFileSync(p, 'latin1')
  });
}

if (!items.length) {
  console.error('Κανένα έγκυρο αρχείο εισόδου.');
  process.exit(1);
}

let res;
try {
  res = M.mergeLevels(items);
} catch (e) {
  console.error('ΣΦΑΛΜΑ ενοποίησης: ' + (e && e.message ? e.message : e));
  process.exit(2);
}

fs.writeFileSync(outPath, res.text, 'latin1');

console.log('--- ΕΝΟΠΟΙΗΣΗ ---');
(res.report || []).forEach(r => {
  console.log('  ' + String(r.file).padEnd(30) +
              ' στάθμη ' + String(r.level).padStart(3) +
              '  -> (' + r.target[0].toFixed(1) + ', ' + r.target[1].toFixed(1) + ')' +
              (r.title ? '  ' + r.title : ''));
});
if (res.warnings && res.warnings.length) {
  console.log('--- ΠΡΟΕΙΔΟΠΟΙΗΣΕΙΣ ---');
  res.warnings.forEach(w => console.log('  ! ' + w));
}
console.log('Γράφτηκε: ' + outPath);
