package lib

import edu.wpi.first.wpilibj.RobotBase
import java.io.File
import java.io.FileWriter
import java.io.PrintWriter
import java.lang.RuntimeException
import java.nio.file.Paths
import java.time.LocalDateTime
import java.time.format.DateTimeFormatter

class Logger(name: String) {

    var writer: PrintWriter? = null

    init {
        try {
            val logFileName = "log-$name-" +
                    "${DateTimeFormatter.ofPattern("yyyy-MM-dd_HH;mm;ss").format(LocalDateTime.now())}.txt"

            val logRoot = if (RobotBase.isReal()) "/home/lvuser/log"
            else File(System.getProperty("user.dir"), "log").absolutePath

            val file = File(Paths.get(logRoot, logFileName).toUri())
            file.parentFile.mkdirs()
            file.createNewFile()

            if (!file.exists()) {
                println("file still doesn't exist!")
                throw RuntimeException("oof")
            }

            writer = PrintWriter(FileWriter(file, true))
        } catch (e: Exception) {
            e.printStackTrace()
            throw e
        }
    }

    fun log(vararg args: Any) {
        val string = args.joinToString()

        writer?.write(string + "\n")
        writer?.flush()
    }

    fun clearLog() {

    }
}
