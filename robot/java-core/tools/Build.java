import javax.tools.*;
import java.io.*;
import java.nio.file.*;
import java.util.*;
import java.util.stream.*;

/** Compiles everything under src/main and src/test into out/ using the JDK compiler (no Gradle needed). */
public class Build {
    public static void main(String[] a) throws Exception {
        List<String> files;
        try (Stream<Path> s = Files.walk(Paths.get("src"))) {
            files = s.filter(p -> p.toString().endsWith(".java")).map(Path::toString).collect(Collectors.toList());
        }
        Files.createDirectories(Paths.get("out"));
        JavaCompiler c = ToolProvider.getSystemJavaCompiler();
        List<String> args = new ArrayList<>(List.of("-d", "out", "-Xlint:all,-options", "-source", "17", "-target", "17"));
        args.addAll(files);
        int rc = c.run(null, null, null, args.toArray(new String[0]));
        System.out.println(rc == 0 ? "BUILD OK (" + files.size() + " files, source+target 17)" : "BUILD FAILED");
        System.exit(rc);
    }
}
