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
     * Reads a CSV file.
     * Uses legacy method for compatibility with older Commons CSV versions on Robot Controller.
     * * @param fileName The absolute path to the file.
     * @return A 2D boolean array representing the CSV data.
     * @throws IOException If the file cannot be read.
     */
    public static boolean[][] getBooleanMatrixFromCsv(String fileName) throws IOException {
        List<boolean[]> rowList = new ArrayList<>();

        // Use the legacy pattern for compatibility
        CSVFormat format = CSVFormat.DEFAULT.withIgnoreSurroundingSpaces(true);
        
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

    public static double[][] getDoubleMatrixFromCsv(String fileName) throws IOException, NumberFormatException {
        List<double[]> rowList = new ArrayList<>();

        // Use the legacy pattern for compatibility
        CSVFormat format = CSVFormat.DEFAULT.withIgnoreSurroundingSpaces(true);

        try (BufferedReader reader = new BufferedReader(new FileReader(fileName));
             CSVParser csvParser = format.parse(reader)) {

            for (CSVRecord record : csvParser) {
                int columnCount = record.size();
                double[] row = new double[columnCount];

                for (int i = 0; i < columnCount; i++) {
                    String cellValue = record.get(i);
                    // Parse double values, throws exception on parse error
                    row[i] = Double.parseDouble(cellValue);
                }
                rowList.add(row);
            }
        }

        // Convert the temporary list to a primitive 2D array
        return rowList.toArray(new double[0][]);
    }

    public static double[] getDoubleArrayFromCsv(String fileName) throws IOException, NumberFormatException{
        List<Double> valueList = new ArrayList<>();

        // Use the legacy pattern for compatibility
        CSVFormat format = CSVFormat.DEFAULT.withIgnoreSurroundingSpaces(true);

        try (BufferedReader reader = new BufferedReader(new FileReader(fileName));
             CSVParser csvParser = format.parse(reader)) {

            for (CSVRecord record : csvParser) {
                for (String cellValue : record) {
                    // Parse double values, throws exception on parse error
                    valueList.add(Double.parseDouble(cellValue));
                }
            }
        }

        // Convert the temporary list to a primitive array
        double[] resultArray = new double[valueList.size()];
        for (int i = 0; i < valueList.size(); i++) {
            resultArray[i] = valueList.get(i);
        }
        return resultArray;
    }
}
