BillieBot Cameo/MSOSA 2022x CSV Import Package
================================================

FILES
-----
1. Blocks.csv
   - 26 Block rows.
   - Required import mapping: Name -> Name.
   - Suggested Target Scope: create/select a package named BillieBotHardwareLibrary.
   - Optional metadata columns can be left unmapped.

2. ValueProperties.csv
   - 318 Value Property rows.
   - Rows are grouped by Owner (the Block name).
   - Core mappings:
       Name          -> Name
       Type          -> Type
       Default Value -> Default Value   (optional; map if exposed by your import dialog)
   - Owner is included so ownership remains explicit and auditable.
   - Unit and RecommendedValueType are retained for a later unit-aware Value Type pass.

RECOMMENDED IMPORT SEQUENCE
---------------------------
A. Create a package in the model named:
      BillieBotHardwareLibrary

B. Import Blocks.csv:
   File > Import From > Excel/CSV File > Import Using New Map
   Element Type: Block
   Target Scope: BillieBotHardwareLibrary
   Map:
      Name -> Name
   Import.

C. Import ValueProperties.csv.

   Safest workflow matching the explicit Cameo 2022x Block/Value Property example:
   - For each unique Owner in ValueProperties.csv, filter/select that Owner's rows.
   - Element Type: Value Property
   - Target Scope: select the corresponding Block
   - Properties to Map: Name, Type, and optionally Default Value
   - Map:
       Name          -> Name
       Type          -> Type
       Default Value -> Default Value   (if exposed)
   - Search references in the entire model so Type references resolve.
   - Import.

   If your MSOSA/Cameo 2022x Excel/CSV Import dialog exposes Owner as a mappable
   property for Value Property elements, you may instead map Owner -> Owner and
   import the combined ValueProperties.csv in one pass. The CSV is structured
   to support that, but the official 2022x Value Property example specifically
   demonstrates a Target Scope per owning Block.

UNIT / VALUE-TYPE HANDLING
--------------------------
- Type is intentionally one of Real, Integer, Boolean, or String so this two-file
  package does not depend on custom Value Types already existing in the model.
- Unit is NOT intended to be mapped as an independent Value Property field.
  In SysML/Cameo, a unit is carried by the SysML Value Type that types the
  Value Property.
- RecommendedValueType records the intended later type, e.g.:
      mass -> Mass_g
      supplyVoltageNominal -> Voltage_V
      rosPublishRate -> Frequency_Hz
- After the basic import is verified, create/reuse unit-bearing SysML Value Types
  (preferably from ISO-80000 where appropriate) and retype numeric properties.

DATA-INTEGRITY NOTES
--------------------
- Every Owner value exactly matches a Block Name in Blocks.csv.
- Every (Owner, Name) pair is unique.
- Value Property names contain no spaces.
- Range/list specifications remain String-valued rather than being silently
  converted into invented scalar min/max properties.
- IdentificationKey and source/traceability columns may be left unmapped.
