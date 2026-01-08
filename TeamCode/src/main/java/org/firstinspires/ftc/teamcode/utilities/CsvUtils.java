package org.firstinspires.ftc.teamcode.utilities;

import org.apache.commons.csv.CSVFormat;
import org.apache.commons.csv.CSVParser;
import org.apache.commons.csv.CSVRecord;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

public class CsvUtils {
    /**
     * Reads a CSV file using the non-deprecated Builder pattern.
     * Optimized for low memory (1GB RAM) on Android API 24.
     * * @param fileName The absolute path to the file.
     * @return A 2D boolean array representing the CSV data.
     * @throws IOException If the file cannot be read.
     */
    public static boolean[][] getBooleanArrayFromCsv(String fileName) throws IOException {
        List<boolean[]> rowList = new ArrayList<>();

        // Use the modern Builder pattern (Non-deprecated)
        CSVFormat format = CSVFormat.DEFAULT.builder()
                .setIgnoreSurroundingSpaces(true) // Built-in trim logic
                .build();

        try (BufferedReader reader = new BufferedReader(new FileReader(fileName));
             CSVParser csvParser = format.parse(reader)) {

            for (CSVRecord record : csvParser) {
                int columnCount = record.size();
                boolean[] row = new boolean[columnCount];

                for (int i = 0; i < columnCount; i++) {
                    String cellValue = record.get(i);
                    // Standard logic for boolean detection
                    row[i] = "true".equalsIgnoreCase(cellValue) || "1".equals(cellValue);
                }
                rowList.add(row);
            }
        }

        // Convert the temporary list to a primitive 2D array
        return rowList.toArray(new boolean[0][]);
    }
}
