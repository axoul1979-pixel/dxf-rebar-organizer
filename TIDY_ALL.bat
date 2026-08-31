@echo off
REM ============================================================
REM  DXF Rebar Auto-Tidy - ΜΑΖΙΚΗ ΕΚΤΕΛΕΣΗ
REM
REM  ΤΡΟΠΟΣ 1 (ευκολότερος):
REM     Σύρε τον ΦΑΚΕΛΟ με τα DXF πάνω σε αυτό το αρχείο.
REM  ΤΡΟΠΟΣ 2:
REM     Διπλό κλικ - θα ζητήσει τη διαδρομή του φακέλου.
REM
REM  Τα αποτελέσματα γραφονται στον ΙΔΙΟ φακελο:
REM     <ονομα>_tidy.dxf   - καθε σχεδιο τακτοποιημενο
REM     _ENOPOIIMENO.dxf   - ΟΛΕΣ οι σταθμες σε ΕΝΑ σχεδιο
REM     _audit.txt / _SUMMARY.txt
REM
REM  Για το _ENOPOIIMENO.dxf χρειαζεται Node.js (nodejs.org).
REM  Χωρις Node βγαινουν κανονικα τα χωριστα _tidy.dxf.
REM ============================================================
setlocal
cd /d "%~dp0"

set TARGET=%~1
if "%TARGET%"=="" (
    set /p TARGET="Διαδρομη φακελου με τα DXF: "
)

if "%TARGET%"=="" (
    echo Δεν δοθηκε φακελος.
    pause
    exit /b 1
)

REM -j 0 = αυτόματο (πυρήνες - 1)
REM Βάλε -t 600 αν θέλεις όριο 10 λεπτών ανά αρχείο.
python batch_parallel.py "%TARGET%"

echo.
echo ============================================
echo  Τελος. Δες το _ENOPOIIMENO.dxf και το _SUMMARY.txt
echo ============================================
pause
