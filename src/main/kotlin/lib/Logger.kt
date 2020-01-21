package lib

import java.io.FileWriter
import java.io.PrintWriter
import java.time.LocalDateTime
import java.time.format.DateTimeFormatter

class Logger(name: String) {

    var writer: PrintWriter? = null

    init {
        try {
            writer = PrintWriter(FileWriter("/home/lvuser/log-$name-" +
                    "${DateTimeFormatter.ofPattern("yyyy=MM-dd_HH-mm-ss").format(LocalDateTime.now())}.txt", false))
        } catch (e: Exception) {
            e.printStackTrace()
        }
    }

    fun log(vararg args: Any) {
        writer?.write(args.joinToString())
    }

}