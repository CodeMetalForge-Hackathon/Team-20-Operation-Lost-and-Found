package c2.app;

import java.nio.file.Path;

/**
 * Start the C2 operations picture.
 *   java -cp out c2.app.Main                      demo with simulated drone
 *   java -cp out c2.app.Main --no-sim             real drone reports only (drop JSON in ./inbox)
 *   options: --inbox=PATH  --out=PATH  --assets=PATH
 */
public class Main {
    public static void main(String[] args) {
        Path inbox = Path.of("inbox"), out = Path.of("out"), assets = Path.of("assets");
        boolean simulate = true;
        for (String arg : args) {
            if (arg.equals("--no-sim")) simulate = false;
            else if (arg.startsWith("--inbox=")) inbox = Path.of(arg.substring(8));
            else if (arg.startsWith("--out=")) out = Path.of(arg.substring(6));
            else if (arg.startsWith("--assets=")) assets = Path.of(arg.substring(9));
        }
        new C2App(inbox, out, assets, simulate).startWithWindow();
    }
}
