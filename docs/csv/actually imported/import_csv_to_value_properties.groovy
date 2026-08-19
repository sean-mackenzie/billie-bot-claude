import com.nomagic.magicdraw.core.Application
import com.nomagic.magicdraw.core.Project
import com.nomagic.magicdraw.openapi.uml.SessionManager
import com.nomagic.uml2.ext.magicdraw.classes.mdkernel.*

import javax.swing.JFileChooser
import javax.swing.filechooser.FileNameExtensionFilter
import java.awt.HeadlessException

def project = Application.getInstance().getProject()
def log = Application.getInstance().getGUILog()
def elementsFactory = project.getElementsFactory()

log.log("=== CSV-driven Value Property Creation with Documentation ===")

// --------------------------------------------------
// Choose input file
// --------------------------------------------------
def file = null
try {
    def chooser = new JFileChooser()
    chooser.setDialogTitle("Select CSV/Text File")
    chooser.setFileFilter(new FileNameExtensionFilter("Text/CSV Files", "csv", "txt"))
    def result = chooser.showOpenDialog(null)

    if (result == JFileChooser.APPROVE_OPTION) {
        file = chooser.getSelectedFile()
    }
} catch (HeadlessException e) {
    log.log("ERROR: File chooser cannot open in headless mode. Please use a GUI session.")
    return
}

if (file == null) {
    log.log("No file selected. Exiting.")
    return
}

log.log("Selected file: " + file.absolutePath)

// --------------------------------------------------
// CSV parser that handles quoted fields
// --------------------------------------------------
def parseCsvLine = { String line ->
    def fields = []
    def current = new StringBuilder()
    boolean inQuotes = false

    for (int i = 0; i < line.length(); i++) {
        char c = line.charAt(i)

        if (c == '"') {
            if (inQuotes && i + 1 < line.length() && line.charAt(i + 1) == '"') {
                current.append('"')
                i++
            } else {
                inQuotes = !inQuotes
            }
        } else if (c == ',' && !inQuotes) {
            fields.add(current.toString().trim())
            current.setLength(0)
        } else {
            current.append(c)
        }
    }

    fields.add(current.toString().trim())
    return fields
}

// --------------------------------------------------
// Helper: get selected model elements from active diagram
// --------------------------------------------------
def getSelectedModelElements = {
    def selectedElements = []

    def activeDiagram = project.getActiveDiagram()
    if (activeDiagram == null) {
        log.log("No active diagram found.")
        return selectedElements
    }

    def selected = activeDiagram.getSelected()
    if (!(selected instanceof Collection) || selected.isEmpty()) {
        log.log("No elements selected on the active diagram.")
        return selectedElements
    }

    selected.each { pe ->
        try {
            def el = pe.getElement()
            if (el != null) {
                selectedElements.add(el)
            }
        } catch (Exception ignored) {
            // Ignore presentation elements that don't map cleanly
        }
    }

    return selectedElements
}

// --------------------------------------------------
// Build a map of selected Block names -> Block element
// --------------------------------------------------
def selectedModelElements = getSelectedModelElements()
def selectedBlocksByName = [:]

selectedModelElements.each { el ->
    if (el instanceof Class) {
        def blockName = el.getName()
        if (blockName != null) {
            selectedBlocksByName[blockName] = (Class) el
        }
    }
}

log.log("Selected block count: " + selectedBlocksByName.size())

if (selectedBlocksByName.isEmpty()) {
    log.log("No selected Blocks found on the active diagram. Exiting.")
    return
}

// --------------------------------------------------
// Helper: find type by name
// --------------------------------------------------
def findTypeByName = { String typeName ->
    def stack = new LinkedList()
    stack.add(project.getModel())

    while (!stack.isEmpty()) {
        def element = stack.removeFirst()

        if (element instanceof NamedElement && element.getName() == typeName) {
            return element
        }

        if (element != null && element.getOwnedElement() != null) {
            stack.addAll(element.getOwnedElement())
        }
    }

    return null
}

// --------------------------------------------------
// Helper: set documentation
// Tries direct documentation setter first, then comment fallback
// --------------------------------------------------
def setElementDocumentation = { NamedElement element, String docText ->
    if (docText == null) {
        return
    }
    def trimmed = docText.trim()
    if (trimmed.length() == 0) {
        return
    }

    // 1) Try direct setter if available
    try {
        if (element.metaClass.respondsTo(element, "setDocumentation", String)) {
            element.setDocumentation(trimmed)
            return
        }
    } catch (Exception ignored) {
        // fall through to comment-based approach
    }

    // 2) Use owned Comment
    try {
        def existingComment = null
        if (element.getOwnedComment() != null && !element.getOwnedComment().isEmpty()) {
            existingComment = element.getOwnedComment().get(0)
        }

        if (existingComment == null) {
            existingComment = elementsFactory.createCommentInstance()
            element.getOwnedComment().add(existingComment)
        }

        existingComment.setBody(trimmed)
    } catch (Exception ex) {
        log.log("WARNING: Could not set documentation on '${element.getName()}': " + ex.message)
    }
}

// --------------------------------------------------
// Main processing
// --------------------------------------------------
def createdCount = 0
def skippedCount = 0
def failedCount = 0
def ignoredCount = 0
def lineCount = 0

SessionManager.getInstance().createSession(project, "Create Value Properties from Selected Diagram Elements")

try {
    file.withReader("UTF-8") { reader ->
        reader.eachLine { rawLine ->
            lineCount++
            def line = rawLine?.trim()

            if (!line) {
                return
            }

            // Skip header row
            if (line.equalsIgnoreCase("BlockName,PropertyName,Type,DefaultValue,Documentation")) {
                log.log("Skipping header row.")
                return
            }

            // Optional comment support
            if (line.startsWith("#")) {
                return
            }

            def fields = parseCsvLine(line)

            if (fields.size() < 5) {
                log.log("SKIP line ${lineCount}: expected 5 fields, got ${fields.size()} -> ${line}")
                skippedCount++
                return
            }

            def blockName = fields[0]
            def propName = fields[1]
            def typeName = fields[2]
            def defaultValueRaw = fields[3]
            def documentationText = fields[4]

            // Only allow blocks currently selected on the diagram
            def block = selectedBlocksByName[blockName]
            if (block == null) {
                log.log("IGNORE line ${lineCount}: Block '${blockName}' is not selected on the active diagram.")
                ignoredCount++
                return
            }

            // Prevent duplicates
            def existing = block.getOwnedAttribute().find { it.getName() == propName }
            if (existing != null) {
                log.log("SKIP line ${lineCount}: '${propName}' already exists on '${blockName}'")
                skippedCount++
                return
            }

            try {
                def prop = elementsFactory.createPropertyInstance()
                prop.setName(propName)
                block.getOwnedAttribute().add(prop)

                // Type assignment
                def typeElement = findTypeByName(typeName)
                if (typeElement != null) {
                    prop.setType(typeElement)
                } else {
                    log.log("WARN line ${lineCount}: Type '${typeName}' not found for '${propName}'")
                }

                // Default value assignment
                switch (typeName) {
                    case "String":
                        def litString = elementsFactory.createLiteralStringInstance()
                        litString.setValue(defaultValueRaw)
                        prop.setDefaultValue(litString)
                        break

                    case "Boolean":
                        def litBool = elementsFactory.createLiteralBooleanInstance()
                        litBool.setValue(Boolean.parseBoolean(defaultValueRaw))
                        prop.setDefaultValue(litBool)
                        break

                    case "Integer":
                        def litInt = elementsFactory.createLiteralIntegerInstance()
                        litInt.setValue(Integer.parseInt(defaultValueRaw))
                        prop.setDefaultValue(litInt)
                        break

                    case "Real":
                        def litReal = elementsFactory.createLiteralRealInstance()
                        litReal.setValue(Double.parseDouble(defaultValueRaw))
                        prop.setDefaultValue(litReal)
                        break

                    default:
                        log.log("WARN line ${lineCount}: Unsupported type '${typeName}' for '${propName}'. Default value not set.")
                }

                // Documentation
                setElementDocumentation(prop, documentationText)

                log.log("CREATED line ${lineCount}: ${blockName}.${propName} : ${typeName} = ${defaultValueRaw}")
                createdCount++
            }
            catch (Exception propEx) {
                log.log("FAIL line ${lineCount}: ${blockName}.${propName} -> ${propEx.message}")
                failedCount++
            }
        }
    }

    SessionManager.getInstance().closeSession(project)

    log.log("=== Selected-diagram import complete ===")
    log.log("Created: ${createdCount}")
    log.log("Skipped: ${skippedCount}")
    log.log("Ignored (not selected): ${ignoredCount}")
    log.log("Failed: ${failedCount}")
    log.log("Selected blocks available: ${selectedBlocksByName.keySet().join(', ')}")
}
catch (Exception ex) {
    SessionManager.getInstance().cancelSession(project)
    log.log("ERROR during import: " + ex.message)
    ex.printStackTrace()
}

log.log("=== End CSV-driven Value Property Creation with Documentation ===")